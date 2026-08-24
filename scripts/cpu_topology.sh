#!/usr/bin/env bash
# cpu_topology.sh — 打印任意 Linux 上位机的 CPU 拓扑（P/E 核识别），
# 用于给不同机器填 cpu_pinning 配置。
#
# 用法: ./scripts/cpu_topology.sh
set -u

echo "== CPU 型号 / 总览 =="
LC_ALL=C lscpu | grep -E "Model name|Socket|Core\(s\) per socket|Thread\(s\) per core|CPU\(s\):"
echo

echo "== 每逻辑核最大频率（P/E 识别依据：频率最高=性能核，其次=能效核）=="
NPROC=$(nproc)
for i in $(seq 0 $(( NPROC - 1 ))); do
  f=$(cat /sys/devices/system/cpu/cpu"$i"/cpufreq/cpuinfo_max_freq 2>/dev/null)
  echo "cpu$i: $(( f / 1000 )) MHz"
done
echo

echo "== 拓扑表（CPU=逻辑核, CORE=物理核, 同一 CORE 的两个 CPU 是超线程兄弟）=="
LC_ALL=C lscpu -e=CPU,CORE,SOCKET,MAXMHZ,MINMHZ 2>/dev/null || LC_ALL=C lscpu -e
echo

echo "== 按频率分组 + 绑核建议 =="
python3 - <<'PYEOF'
import glob
import re
import collections

info = {}          # cpu -> (max_mhz, core_id)
for p in glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq/cpuinfo_max_freq'):
    m = re.search(r'/cpu(\d+)/', p)
    if not m:
        continue
    cpu = int(m.group(1))
    raw = open(p).read().strip()
    if not raw:
        continue  # 偶发空文件直接跳过
    try:
        core_id = open(f'/sys/devices/system/cpu/cpu{cpu}/topology/core_id').read().strip()
    except OSError:
        core_id = None
    info[cpu] = (int(raw) // 1000, core_id)

if not info:
    raise SystemExit('未找到 CPU 频率信息')

# 识别方法：Intel 混合架构上 P 核有超线程（同一物理核 2 个逻辑核），E 核没有。
core_threads = collections.defaultdict(list)
for cpu, (mhz, cid) in info.items():
    if cid is not None:
        core_threads[cid].append(cpu)
ht_cores = {cid for cid, cpus in core_threads.items() if len(cpus) >= 2}

p_cpus = sorted(c for c, (m, cid) in info.items() if cid in ht_cores)
e_cpus = sorted(c for c, (m, cid) in info.items() if cid is not None and cid not in ht_cores)

def rng(cpus):
    return f"{cpus[0]}-{cpus[-1]}" if len(cpus) > 1 else str(cpus[0])

def freq_of(cpus):
    return sorted({info[c][0] for c in cpus}, reverse=True)

if e_cpus:
    print(f"  P核(性能, 含超线程): {len(p_cpus)} 逻辑核 -> {rng(p_cpus)}  最大频率 {freq_of(p_cpus)} MHz")
    print(f"  E核(能效, 无超线程): {len(e_cpus)} 逻辑核 -> {rng(e_cpus)}  最大频率 {freq_of(e_cpus)} MHz")
    phys_p = len({info[c][1] for c in p_cpus})
    print()
    print("  cpu_pinning 建议:")
    print(f"    other_cores: \"{rng(e_cpus)}\"")
    print(f"    yolo_core_type: \"pcore\"      # 推理用 P 核 {rng(p_cpus)}")
    print(f"    RP24_YOLO_infer_threads: {phys_p}   # 关超线程(HT off)；开HT可试 {phys_p * 2}")
else:
    # 全部核都有超线程（或读不到 core_id）：退回按最大频率分组，最高频那组当作 P
    groups = collections.defaultdict(list)
    for cpu, (mhz, cid) in info.items():
        groups[mhz].append(cpu)
    sorted_freq = sorted(groups, reverse=True)
    if len(sorted_freq) >= 2:
        p = sorted(groups[sorted_freq[0]])
        e = sorted(groups[sorted_freq[1]])
        print(f"  无法按超线程区分，退回频率分组：P {rng(p)} ({sorted_freq[0]}MHz) / 其余 {rng(e)} ({sorted_freq[1]}MHz)")
        print(f"  cpu_pinning 建议: other_cores \"{rng(e)}\", yolo_core_type \"pcore\", infer_threads {len(p) // 2}")
    else:
        print(f"  全部 {len(info)} 逻辑核同频 {sorted_freq[0]} MHz：无 P/E 之分，cpu_pinning.enabled 建议 false")
PYEOF
