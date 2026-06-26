# Mini Feedforward Cascade Ratio Control（前馈-串级-比值控制）

**从零开始、零依赖的 C 语言实现**，涵盖工业过程控制中的高级控制策略：前馈、串级、比值、分程、超驰/选择器、死区补偿、MIMO 解耦以及增益调度 PID。每个模块将控制理论教科书中的数学设计流程转化为可运行的 C 代码，实现理论与工程实践的桥接。

## 子模块总览

| 子模块 | 主题 | 参考课程 |
|--------|------|----------|
| [mini-cascade-control-primary-secondary](mini-cascade-control-primary-secondary/) | 串级 PID 控制，内/外环整定（ZN、Cohen-Coon、SIMC、Lambda），无扰切换，Nyquist 稳定裕度，前馈补偿 | MIT 6.241J, MIT 2.151 |
| [mini-deadtime-compensation-smith](mini-deadtime-compensation-smith/) | Smith 预估器（FOPDT/SOPDT），无延迟预测，模型失配校正，自适应 Smith，灵敏度/鲁棒性分析，蒙特卡洛验证 | MIT 6.241J, Åström–Hägglund |
| [mini-decoupling-mimo-process](mini-decoupling-mimo-process/) | 静态/动态/逆向/SVD 解耦，RGA 交互分析，Niederlinski 指数，条件数，Wood-Berry 蒸馏模型，鲁棒稳定性（μ 分析） | MIT 6.241J, MIT 2.151, Skogestad |
| [mini-gain-scheduled-pid](mini-gain-scheduled-pid/) | 基于调度变量的增益调度，插值方法（最近邻/线性/Hermite/样条/RBF），PID 形式（理想/串联/ISA/二自由度），稳定性分析，自适应扩展 | MIT 6.241J, MIT 16.30 |
| [mini-override-selector-control](mini-override-selector-control/) | 超驰/选择器控制，约束管理，信号选择（高选/低选/中选），阀位控制（VPC），带抗积分饱和跟踪的 PID，诊断 | MIT 6.241J, ISA-5.1 |
| [mini-ratio-control-gas-liquid](mini-ratio-control-gas-liquid/) | 主从比值跟踪，交叉限幅燃烧安全，气/液流量温压补偿，混合优化，自适应比值微调（RLS） | MIT 6.241J, MIT 10.450 |
| [mini-split-range-control-heat-cool](mini-split-range-control-heat-cool/) | 分程映射（加热/冷却），阀门特性化，高级自整定，自适应增益，反应器安全排序 | MIT 6.241J, MIT 10.450 |
| [mini-static-dynamic-feedforward](mini-static-dynamic-feedforward/) | 静态/动态前馈，超前-滞后补偿，FF+FB 联合控制，迭代学习控制（ILC），卡尔曼滤波扰动估计，非最小相位分解 | MIT 6.241J, MIT 2.151 |

## 设计理念

- **零外部依赖** — 纯 C（C99/C11），仅使用 `libc` 和 `libm`
- **模块自包含** — 每个目录自带 `Makefile`、`include/`、`src/`、`examples/`、`demos/`、`tests/`
- **理论到代码的映射** — 每个模块包含 `docs/` 目录，内有课程对齐说明，将教科书中的方框图转化为可运行的控制循环
- **工业级数据结构** — 对标真实 DCS/PLC 控制功能块：PID 形式、过程模型、传感器/执行器接口

## 构建方式

每个模块相互独立。进入模块目录后运行：

```bash
cd mini-cascade-control-primary-secondary
make all    # 构建全部
make test   # 运行测试
```

需要 **GCC** 和 **GNU Make**。

## 项目结构

```
mini-feedforward-cascade-ratio/
├── mini-cascade-control-primary-secondary/   # 串级 PID — 主/副回路控制
├── mini-deadtime-compensation-smith/          # Smith 预估器 — 死区时间过程补偿
├── mini-decoupling-mimo-process/              # MIMO 解耦与交互分析
├── mini-gain-scheduled-pid/                   # 增益调度 PID — 跨工况自适应
├── mini-override-selector-control/            # 超驰/选择器 — 约束处理
├── mini-ratio-control-gas-liquid/             # 比值控制 — 交叉限幅安全联锁
├── mini-split-range-control-heat-cool/        # 分程控制 — 多阀门顺序动作
└── mini-static-dynamic-feedforward/           # 静态与动态前馈补偿
```

## 许可证

MIT
