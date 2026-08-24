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

groups = {}
for p in glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq/cpuinfo_max_freq'):
    m = re.search(r'/cpu(\d+)/', p)
    if not m:
        continue
    cpu = int(m.group(1))
    raw = open(p).read().strip()
    if not raw:
        continue  # 偶发空文件直接跳过
    mhz = int(raw) // 1000
    groups.setdefault(mhz, []).append(cpu)

if not groups:
    raise SystemExit('未找到 CPU 频率信息')

sorted_freq = sorted(groups, reverse=True)
for i, mhz in enumerate(sorted_freq):
    cpus = sorted(groups[mhz])
    rng = f"{cpus[0]}-{cpus[-1]}" if len(cpus) > 1 else str(cpus[0])
    if i == 0:
        role = "P核(性能)"
    elif len(sorted_freq) > 1 and i == 1:
        role = "E核(能效)"
    else:
        role = "低功耗E核"
    print(f"  {mhz} MHz  {len(cpus)} 核: {rng}  <- {role}")

if len(sorted_freq) >= 2:
    p = sorted(groups[sorted_freq[0]])
    e = sorted(groups[sorted_freq[1]])
    # 物理 P 核数 = P 组里不同的 core_id 个数（去超线程重复）
    phys_p = set()
    for c in p:
        try:
            phys_p.add(open(f'/sys/devices/system/cpu/cpu{c}/topology/core_id').read().strip())
        except OSError:
            pass
    print()
    print("  cpu_pinning 建议:")
    print(f"    other_cores: \"{e[0]}-{e[-1]}\"")
    print(f"    yolo_core_type: \"pcore\"      # 推理用 P 核 {p[0]}-{p[-1]}")
    print(f"    RP24_YOLO_infer_threads: {len(phys_p)}   # 关超线程(HT off)；开HT可试 {len(phys_p) * 2}")
else:
    print()
    print("  只有一组频率：该 CPU 无 P/E 之分，cpu_pinning.enabled 建议保持 false")
PYEOF
