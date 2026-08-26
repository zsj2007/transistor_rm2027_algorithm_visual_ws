#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
modify_0526_dynamic.py — 将静态 640×640 / batch=1 的魔改 YOLO 模型 0526.onnx 改造为
动态 batch + 动态输入分辨率模型（0526_dynamic.onnx）。

背景
----
Model/Outpost/0526.onnx 是一个由 PyTorch 2.0.1 静态导出的"魔改 YOLO"（无原始 .pt 文件，
头部为 4 关键点 landmarks + conf + 4 色 + 9 类，共 22 通道），导出时把输入分辨率写死为
[1, 3, 640, 640]、输出写死为 [1, 25200, 22]（25200 = 3 尺度 × 3 anchor × (80²+40²+20²)）。
因此：
  * 把 config 的 outpost.inference.max_batch 调大后，OpenVINO 按 batch=2/3/4 reshape 编译
    会失败（检测头 Reshape 目标形状写死 batch=1），InferCore 只能回退到 batch=1；
  * 改用 512×512 / 320×320 输入会在检测头形状上直接报错。

改造方法（本脚本实现，共 4 步）
--------------------------------
1. 输入/输出形状动态化
   输入 images:  [1, 3, 640, 640]  ->  ['batch', 3, 'height', 'width']
   输出 output:  [1, 25200, 22]    ->  ['batch', 'num_anchors', 22]

2. 检测头 Reshape 目标形状动态化（3 个尺度各 2 处，共 6 处）
   原静态目标：
     Reshape#1  [1, 3, 22, Hs, Ws]（由特征图 [B, 66, Hs, Ws] 重排）
     Reshape#2  [1, 3*Hs*Ws, 22]（最终展平，供 Concat 拼成 output）
   替换为运行时用 Shape/Gather/Concat 现算（与 power_rune.onnx 的导出风格一致）：
     t1 = Concat([B], [3], [22], [Hs], [Ws])
     t2 = Concat([B], [3*Hs*Ws], [22])
   注意：Gather 的 indices 必须用 0-D 标量，使 Gather(Shape(x), idx) 输出 0-D 标量，
   否则 onnxruntime 加载时报 "Input to 'Range' op should be scalars"。

3. 网格偏移（grid offsets）动态化（3 个尺度各 1 处 Add）
   原图内嵌了 YOLO 式解码：对每个尺度把 4 个关键点的 xy 加上网格偏移
   (gx*stride, gy*stride)，偏移常数形状为 [1, 1, Hs, Ws, 22]（stride 分别为 8/16/32），
   分辨率一变这些常数就失效（形状无法广播）。改为运行时生成：
     xs = Range(0, Ws, 1) * stride   -> [Ws]
     ys = Range(0, Hs, 1) * stride   -> [Hs]
     grid = x_mask * xs[1,1,1,Ws,1] + y_mask * ys[1,1,Hs,1,1]   （掩码只在 xy 通道为 1）
   在 640×640 下与原静态常数逐位一致（已验证 bitwise 相同）。

4. 删除导出的静态 value_info
   原模型带 217 条 torch 导出时记录的中间张量形状（全部写死 640×640 / batch=1）。
   onnx / onnxruntime 的 shape inference 会优先采用这些已声明形状，导致即便输入已声明
   动态，整个图仍被解析成 batch=1 / 640×640，触发常量折叠把动态子图折叠回静态值。
   删除全部 value_info 后形状推断完全由动态输入推导。

用法
----
  python3 modify_0526_dynamic.py [--input Model/Outpost/0526.onnx]
                                [--output Model/Outpost/0526_dynamic.onnx]
                                [--no-verify]

配套改动
--------
C++ 侧后处理原先把 anchor 数写死为 NUM_ANCHORS=25200（include/Outpost/OpenvinoInfer.h），
分辨率一改就会拒绝输出。配合本脚本需将后处理改为按输出张量形状动态读取 anchor 数
（已随分支提交：删除 NUM_ANCHORS，postprocess 内用 shape[1] 作为循环/步长）。

验证
----
脚本默认对输出模型做：
  * onnx.checker.check_model（full_check）
  * onnxruntime 冒烟测试：batch=1/2/4 × 640/512/320，检查输出形状
  * 与原模型在 640×640 batch=1 下逐位一致对比（要求 max_abs_diff == 0）

依赖：onnx、numpy、onnxruntime（仅验证阶段需要 onnxruntime）。
"""
import argparse
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

# ---------------------------------------------------------------------------
# 模型专有常量（与 0526.onnx 对齐；如模型结构不同请按需修改）
# ---------------------------------------------------------------------------
# 每个尺度的检测头：特征图来自 m.model.24.m.<i> 的 1x1 卷积，输出 [B, 66, Hs, Ws]
#   conv     —— 卷积输出（Reshape#1 的数据输入，且在其之前产生）
#   reshape1 —— Reshape#1 输出（[B, 3, 22, Hs, Ws] 的 5D 重排）
#   mul      —— Cast 后乘 anchor 常量的输出（[B, 3, Hs, Ws, 22]，Add 的数据输入）
#   add      —— 网格偏移 Add 输出（Reshape#2 的数据输入）
#   reshape2 —— Reshape#2 输出（[B, 3*Hs*Ws, 22]）
#   stride   —— 该尺度下采样倍数（网格偏移步长）
SCALES = [
    dict(conv="/m/model.24/m.0/Conv_output_0", reshape1="/m/model.24/Reshape_output_0",
         mul="/m/model.24/Mul_output_0",       add="/m/model.24/Add_output_0",
         reshape2="/m/model.24/Reshape_1_output_0", stride=8.0, tag="s0"),
    dict(conv="/m/model.24/m.1/Conv_output_0", reshape1="/m/model.24/Reshape_2_output_0",
         mul="/m/model.24/Mul_1_output_0",     add="/m/model.24/Add_1_output_0",
         reshape2="/m/model.24/Reshape_3_output_0", stride=16.0, tag="s1"),
    dict(conv="/m/model.24/m.2/Conv_output_0", reshape1="/m/model.24/Reshape_4_output_0",
         mul="/m/model.24/Mul_2_output_0",     add="/m/model.24/Add_2_output_0",
         reshape2="/m/model.24/Reshape_5_output_0", stride=32.0, tag="s2"),
]
# 通道布局：22 = 8(4 点 xy) + 1(conf) + 4(color) + 9(class)；xy 通道 0,2,4,6 加 gx*stride，
# 1,3,5,7 加 gy*stride，其余通道加 0
X_CHANNELS = (0, 2, 4, 6)
Y_CHANNELS = (1, 3, 5, 7)
OUT_DIM = 22
NUM_ANCHORS_640 = 25200  # 原模型输出 anchor 数（校验用）

# 将被删除的静态 initializer（reshape 目标 + 网格偏移常数）
STATIC_INITS_TO_DROP = [
    "/m/model.24/Constant_output_0", "/m/model.24/Constant_3_output_0",
    "/m/model.24/Constant_4_output_0", "/m/model.24/Constant_8_output_0",
    "/m/model.24/Constant_2_output_0", "/m/model.24/Constant_6_output_0",
    "/m/model.24/Constant_10_output_0",
]


def _dim(d, value=None, param=None):
    """构造 TensorShapeProto.Dimension：value 与 param 二选一。"""
    if value is not None:
        d.ClearField("dim_param")
        d.dim_value = value
    elif param is not None:
        d.ClearField("dim_value")
        d.dim_param = param
    return d


def make_dynamic_input_output(model):
    """步骤 1：输入/输出形状动态化。"""
    inp = model.graph.input[0]
    assert inp.name == "images", f"expected input 'images', got {inp.name}"
    d = inp.type.tensor_type.shape.dim
    assert len(d) == 4, d
    assert d[0].dim_value == 1 and d[2].dim_value == 640 and d[3].dim_value == 640, \
        "input is not [1,3,640,640] — model structure differs from 0526.onnx"
    _dim(d[0], param="batch")
    _dim(d[2], param="height")
    _dim(d[3], param="width")

    out = model.graph.output[0]
    assert out.name == "output", f"expected output 'output', got {out.name}"
    od = out.type.tensor_type.shape.dim
    assert len(od) == 3 and od[0].dim_value == 1 and od[1].dim_value == NUM_ANCHORS_640 \
        and od[2].dim_value == OUT_DIM, "output is not [1,25200,22]"
    _dim(od[0], param="batch")
    _dim(od[1], param="num_anchors")


def build_dynamic_subgraphs(model, scales):
    """步骤 2+3：为每个尺度构建动态 reshape 目标子图与动态网格偏移子图。"""
    g = model.graph
    nodes_by_out = {n.output[0]: n for n in g.node}
    for s in scales:
        assert nodes_by_out[s["reshape1"]].op_type == "Reshape", s
        assert nodes_by_out[s["reshape2"]].op_type == "Reshape", s
        assert nodes_by_out[s["add"]].op_type == "Add", s

    # ---- 共享 initializer（0-D 标量给 Gather/Range 用，[1] 给 Concat 用）----
    shared_inits = [
        numpy_helper.from_array(np.int64(0), name="/dyn/idx0"),
        numpy_helper.from_array(np.int64(2), name="/dyn/idx2"),
        numpy_helper.from_array(np.int64(3), name="/dyn/idx3"),
        numpy_helper.from_array(np.int64(3), name="/dyn/three_s"),          # 0-D，Mul 用
        numpy_helper.from_array(np.array([3], dtype=np.int64), name="/dyn/three"),      # [1]，Concat 用
        numpy_helper.from_array(np.array([OUT_DIM], dtype=np.int64), name="/dyn/twentytwo"),
        numpy_helper.from_array(np.float32(0.0), name="/dyn/zero"),
        numpy_helper.from_array(np.float32(1.0), name="/dyn/one"),
    ]
    xmask = np.zeros((1, 1, 1, 1, OUT_DIM), dtype=np.float32)
    ymask = np.zeros((1, 1, 1, 1, OUT_DIM), dtype=np.float32)
    for c in X_CHANNELS:
        xmask[0, 0, 0, 0, c] = 1.0
    for c in Y_CHANNELS:
        ymask[0, 0, 0, 0, c] = 1.0
    shared_inits += [
        numpy_helper.from_array(xmask, name="/dyn/xmask"),
        numpy_helper.from_array(ymask, name="/dyn/ymask"),
    ]

    blockA, blockB, per_scale_inits = {}, {}, []
    for s in scales:
        tag, F, T = s["tag"], s["conv"], s["mul"]
        # ---- Block A：Reshape#1 的目标 [B, 3, 22, Hs, Ws]，取自 Shape(F) ----
        # 必须用 F（卷积输出，在 Reshape#1 之前产生）；T 产生于 Reshape#1 之后，不能用。
        A = [
            helper.make_node("Shape", [F], [tag + "/shapeF"], name=tag + "/ShapeF"),
            helper.make_node("Gather", [tag + "/shapeF", "/dyn/idx0"], [tag + "/bF"], axis=0, name=tag + "/GatherBF"),
            helper.make_node("Gather", [tag + "/shapeF", "/dyn/idx2"], [tag + "/hF"], axis=0, name=tag + "/GatherHF"),
            helper.make_node("Gather", [tag + "/shapeF", "/dyn/idx3"], [tag + "/wF"], axis=0, name=tag + "/GatherWF"),
            helper.make_node("Unsqueeze", [tag + "/bF"], [tag + "/bF1"], axes=[0], name=tag + "/UnsqueezeBF"),
            helper.make_node("Unsqueeze", [tag + "/hF"], [tag + "/hF1"], axes=[0], name=tag + "/UnsqueezeHF"),
            helper.make_node("Unsqueeze", [tag + "/wF"], [tag + "/wF1"], axes=[0], name=tag + "/UnsqueezeWF"),
            helper.make_node("Concat", [tag + "/bF1", "/dyn/three", "/dyn/twentytwo", tag + "/hF1", tag + "/wF1"],
                             [tag + "/t1"], axis=0, name=tag + "/ConcatT1"),
        ]
        # ---- Block B：动态网格偏移 + Reshape#2 的目标 [B, 3*Hs*Ws, 22]，取自 Shape(T) ----
        B = [
            helper.make_node("Shape", [T], [tag + "/shape"], name=tag + "/Shape"),
            helper.make_node("Gather", [tag + "/shape", "/dyn/idx0"], [tag + "/b"], axis=0, name=tag + "/GatherB"),
            helper.make_node("Gather", [tag + "/shape", "/dyn/idx2"], [tag + "/h"], axis=0, name=tag + "/GatherH"),
            helper.make_node("Gather", [tag + "/shape", "/dyn/idx3"], [tag + "/w"], axis=0, name=tag + "/GatherW"),
            helper.make_node("Cast", [tag + "/w"], [tag + "/w_f"], to=TensorProto.FLOAT, name=tag + "/CastW"),
            helper.make_node("Cast", [tag + "/h"], [tag + "/h_f"], to=TensorProto.FLOAT, name=tag + "/CastH"),
            helper.make_node("Range", ["/dyn/zero", tag + "/w_f", "/dyn/one"], [tag + "/xs"], name=tag + "/RangeX"),
            helper.make_node("Range", ["/dyn/zero", tag + "/h_f", "/dyn/one"], [tag + "/ys"], name=tag + "/RangeY"),
            helper.make_node("Mul", [tag + "/xs", tag + "/stride"], [tag + "/xoff"], name=tag + "/MulX"),
            helper.make_node("Mul", [tag + "/ys", tag + "/stride"], [tag + "/yoff"], name=tag + "/MulY"),
            helper.make_node("Unsqueeze", [tag + "/xoff"], [tag + "/xoff5"], axes=[0, 1, 2, 4], name=tag + "/UnsqueezeX"),
            helper.make_node("Unsqueeze", [tag + "/yoff"], [tag + "/yoff5"], axes=[0, 1, 3, 4], name=tag + "/UnsqueezeY"),
            helper.make_node("Mul", [tag + "/xoff5", "/dyn/xmask"], [tag + "/xpart"], name=tag + "/MaskX"),
            helper.make_node("Mul", [tag + "/yoff5", "/dyn/ymask"], [tag + "/ypart"], name=tag + "/MaskY"),
            helper.make_node("Add", [tag + "/xpart", tag + "/ypart"], [tag + "/grid"], name=tag + "/GridSum"),
            helper.make_node("Mul", [tag + "/h", tag + "/w"], [tag + "/hw"], name=tag + "/MulHW"),
            helper.make_node("Mul", [tag + "/hw", "/dyn/three_s"], [tag + "/n"], name=tag + "/MulN"),
            helper.make_node("Unsqueeze", [tag + "/b"], [tag + "/b1"], axes=[0], name=tag + "/UnsqueezeB"),
            helper.make_node("Unsqueeze", [tag + "/n"], [tag + "/n1"], axes=[0], name=tag + "/UnsqueezeN"),
            helper.make_node("Concat", [tag + "/b1", tag + "/n1", "/dyn/twentytwo"], [tag + "/t2"], axis=0, name=tag + "/ConcatT2"),
        ]
        per_scale_inits.append(numpy_helper.from_array(np.float32(s["stride"]), name=tag + "/stride"))
        blockA[tag], blockB[tag] = A, B

    # ---- 改写消费者：Reshape#1 用 t1，Add 用 grid，Reshape#2 用 t2 ----
    nodes = list(g.node)
    for s in scales:
        nodes_by_out[s["reshape1"]].input[1] = s["tag"] + "/t1"
        nodes_by_out[s["add"]].input[1] = s["tag"] + "/grid"
        nodes_by_out[s["reshape2"]].input[1] = s["tag"] + "/t2"

    def insert_before(nodes, block, before_output_name):
        idx = next(i for i, n in enumerate(nodes) if n.output[0] == before_output_name)
        return nodes[:idx] + block + nodes[idx:]

    for s in scales:  # 按原图顺序插入，保持拓扑有序
        nodes = insert_before(nodes, blockA[s["tag"]], s["reshape1"])
        nodes = insert_before(nodes, blockB[s["tag"]], s["add"])

    del g.node[:]
    g.node.extend(nodes)
    g.initializer.extend(shared_inits)
    g.initializer.extend(per_scale_inits)


def drop_static_initializers(model):
    """删除不再使用的静态 reshape 目标与网格偏移 initializer。"""
    g = model.graph
    keep = [i for i in g.initializer if i.name not in STATIC_INITS_TO_DROP]
    del g.initializer[:]
    g.initializer.extend(keep)


def strip_value_info(model):
    """步骤 4：删除 torch 静态导出的 value_info（写死的中间形状会压过动态输入）。"""
    del model.graph.value_info[:]


# ---------------------------------------------------------------------------
# 验证
# ---------------------------------------------------------------------------
def verify(model, with_ort=True):
    """onnx checker + ORT 冒烟测试 + 与原模型 640 batch=1 逐位一致。"""
    onnx.checker.check_model(model, full_check=True)
    print("[verify] onnx.checker full_check OK")

    if not with_ort:
        return
    try:
        import onnxruntime as ort
    except ImportError:
        print("[verify] onnxruntime 未安装，跳过运行时验证")
        return

    tmp = "/tmp/_0526_dynamic_verify.onnx"
    onnx.save(model, tmp)
    sess = ort.InferenceSession(tmp, providers=["CPUExecutionProvider"])

    # 输出形状声明
    oshp = sess.get_outputs()[0].shape
    print(f"[verify] ORT 输出声明: {oshp}")

    # 冒烟：batch=1/2/4 × 640/512/320
    cases = [(1, 640), (2, 640), (4, 640), (1, 512), (2, 512), (1, 320), (4, 320)]
    for bs, hw in cases:
        x = np.random.rand(bs, 3, hw, hw).astype(np.float16)
        out = sess.run(None, {"images": x})[0]
        # 期望 anchor 数：每尺度 3 anchor，共 3 尺度 → 3 * ((s/8)² + (s/16)² + (s/32)²)
        expect = 3 * ((hw // 8) ** 2 + (hw // 16) ** 2 + (hw // 32) ** 2)
        assert out.shape == (bs, expect, OUT_DIM), (bs, hw, out.shape, expect)
        print(f"[verify] batch={bs} {hw}x{hw} -> {out.shape} OK")

    # 与原模型 640×640 batch=1 逐位一致
    sess_orig = ort.InferenceSession(orig_model_path_global, providers=["CPUExecutionProvider"])
    np.random.seed(0)
    x = np.random.rand(1, 3, 640, 640).astype(np.float16)
    a = sess_orig.run(None, {"images": x})[0]
    b = sess.run(None, {"images": x})[0]
    diff = float(np.max(np.abs(a - b)))
    bitwise = np.array_equal(a.view(np.uint32), b.view(np.uint32))
    print(f"[verify] 640x640 batch=1 max_abs_diff={diff:.6g} bitwise_equal={bitwise}")
    assert bitwise, "与原模型输出不一致！"


orig_model_path_global = None  # 由 main 注入，供 verify 使用


def main():
    ap = argparse.ArgumentParser(description="把 0526.onnx 改造为动态 batch + 动态分辨率")
    ap.add_argument("--input", default="Model/Outpost/0526.onnx")
    ap.add_argument("--output", default="Model/Outpost/0526_dynamic.onnx")
    ap.add_argument("--no-verify", action="store_true", help="跳过运行时验证")
    args = ap.parse_args()

    global orig_model_path_global
    orig_model_path_global = args.input

    print(f"[1/4] 加载 {args.input} ...")
    model = onnx.load(args.input)
    make_dynamic_input_output(model)

    print("[2/4] 检测头 reshape 目标 + 网格偏移动态化 ...")
    build_dynamic_subgraphs(model, SCALES)
    drop_static_initializers(model)

    print("[3/4] 删除静态 value_info（写死的 640/batch=1 中间形状）...")
    strip_value_info(model)

    print(f"[4/4] 保存 {args.output} ...")
    onnx.save(model, args.output)
    print("完成。输出模型输入/输出：")
    print("  input :", [(d.dim_param or d.dim_value) for d in model.graph.input[0].type.tensor_type.shape.dim])
    print("  output:", [(d.dim_param or d.dim_value) for d in model.graph.output[0].type.tensor_type.shape.dim])

    if not args.no_verify:
        print("\n开始验证 ...")
        verify(model, with_ort=True)
        print("验证全部通过。")


if __name__ == "__main__":
    sys.exit(main())
