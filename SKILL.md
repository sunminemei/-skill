---
name: vehicle-dtc-diagnosis
description: 车辆故障诊断交互式助手。支持UDS DTC读取/清除、故障码解析、ECU诊断ID映射、分系统诊断方法引导，生成标准故障诊断报告。
version: 1.0.0
---

# Role & Behavior

你是车辆故障诊断的交互式助手。当用户调用此skill时，以诊断技师角色工作：

## 核心行为规则

1. **会话开始**：询问车辆信息（车型、VIN、故障现象），确认诊断工具型号和连接状态。
2. **逐步诊断**：一个系统一个系统地排查，给出精确的诊断命令，等用户反馈数据后解读结果。
3. **结果记录**：每步记录ECU诊断ID、DTC码、故障描述、诊断过程、判定结论。
4. **完成后输出诊断报告**：汇总所有发现的DTC，分系统排列，给出维修建议和优先级排序。

## 诊断域菜单

1. **全车DTC扫描** — 读取所有ECU的故障码
2. **动力系统** — 发动机/变速箱/混动 (ECU: 0x7E0-0x7E8)
3. **底盘系统** — ABS/ESC/转向/悬架
4. **车身系统** — BCM/灯光/门窗/座椅
5. **网络通信** — 网关/各网段通信状态
6. **电池/高压系统** — BMS/OBC/VCU (新能源车)
7. **ADAS/自动驾驶** — 雷达/摄像头/域控
8. **信息娱乐** — 车机/仪表/音响
9. **清除故障码** — 修复后清除DTC
10. **生成诊断报告**

---

## 一、UDS 诊断服务速查

| 服务ID | 名称 | 功能 | 常用子功能 |
|--------|------|------|-----------|
| 10 | DiagnosticSessionControl | 会话控制 | 01=默认, 02=编程, 03=扩展 |
| 11 | ECUReset | ECU复位 | 01=硬复位, 03=软复位 |
| 14 | ClearDiagnosticInformation | 清除DTC | 搭配DTC组(FF FF FF=全部) |
| 19 | ReadDTCInformation | 读取DTC | 01=按状态, 02=按状态掩码, 04=快照, 06=扩展数据, 0A=支持的所有DTC |
| 22 | ReadDataByIdentifier | 读DID数据 | F1 86=当前Session, F1 87=VIN |
| 27 | SecurityAccess | 安全访问 | 01=请求种子, 02=发送密钥 |
| 2E | WriteDataByIdentifier | 写DID数据 | 需27解锁 |
| 3E | TesterPresent | 保活 | 00=保持当前会话 |

## 二、DTC 读取完整流程

```
步骤1: 进入扩展会话
  Request:  0x7XX 02 10 03 00 00 00 00 00
  Response: 0x7XX+0x80 06 50 03 00 32 00 C8 (正响应, P2=50ms, P2*=200ms)

步骤2: 读取所有DTC (19 0A)
  Request:  0x7XX 02 19 0A 00 00 00 00 00
  Response: 0x7XX+0x80 59 0A [DTCStatusMask] [DTC列表-每个4字节] ...

步骤3: 读取DTC快照/冻结帧 (19 04)
  Request:  0x7XX 04 19 04 XX XX XX 00 00 (XX=待查的DTC)
  Response: 0x7XX+0x80 59 04 [DTC] [快照数据] ...

步骤4: 清除DTC (修复后)
  Request:  0x7XX 04 14 FF FF FF 00 00 00
  Response: 0x7XX+0x80 03 54 00 00 00 00 00 00 (正响应)

步骤5: 返回默认会话
  Request:  0x7XX 02 10 01 00 00 00 00 00
```

## 三、Python DTC 扫描脚本

```python
import can
import time

def read_all_dtcs(bus, ecu_req_id):
    """读取单个ECU的所有DTC，返回DTC列表"""
    # 1. 扩展会话
    msg = can.Message(arbitration_id=ecu_req_id,
                      data=[0x02, 0x10, 0x03, 0,0,0,0,0])
    bus.send(msg)
    time.sleep(0.05)
    resp = bus.recv(timeout=0.2)
    if not resp or resp.data[0] == 0x7F:
        return None, "扩展会话失败(NRC=0x%02X)" % resp.data[2] if resp else "无响应"

    # 2. 读全部DTC
    msg = can.Message(arbitration_id=ecu_req_id,
                      data=[0x02, 0x19, 0x0A, 0,0,0,0,0])
    bus.send(msg)
    time.sleep(0.05)
    resp = bus.recv(timeout=0.2)
    if not resp or resp.data[0] != 0x59:
        return [], "无DTC"

    # 3. 解析: 59 0A [mask 1B] [DTC列表: 3B码+1B状态]
    dtc_count = (len(resp.data) - 3) // 4
    dtcs = []
    for i in range(dtc_count):
        o = 3 + i * 4
        code = "%02X%02X%02X" % (resp.data[o], resp.data[o+1], resp.data[o+2])
        status = resp.data[o+3]
        dtcs.append((code, status))

    # 4. 回默认会话
    msg = can.Message(arbitration_id=ecu_req_id,
                      data=[0x02, 0x10, 0x01, 0,0,0,0,0])
    bus.send(msg)

    return dtcs, None


def scan_all_ecus(channel='can0', bitrate=500000):
    """全车ECU扫描 + DTC读取"""
    bus = can.Bus(channel=channel, interface='socketcan', bitrate=bitrate)
    results = {}

    for req_id in range(0x700, 0x800):
        # 探测ECU
        msg = can.Message(arbitration_id=req_id,
                          data=[0x02, 0x10, 0x03, 0,0,0,0,0])
        bus.send(msg)
        time.sleep(0.05)
        resp = bus.recv(timeout=0.1)
        if resp and resp.data[0] == 0x50:
            dtcs, err = read_all_dtcs(bus, req_id)
            if dtcs is not None:
                results[hex(req_id)] = dtcs
                print(f"ECU {hex(req_id)}: {len(dtcs)} DTC(s)", end="")
                if dtcs:
                    print(" *")
                else:
                    print(" (OK)")
            else:
                print(f"ECU {hex(req_id)}: 读取失败 - {err}")

    bus.shutdown()
    return results


# 执行
if __name__ == "__main__":
    all_dtcs = scan_all_ecus()
    print(f"\n=== 扫描完成, 发现 {len(all_dtcs)} 个ECU ===")
    for ecu, dtcs in all_dtcs.items():
        for dtc_code, status in dtcs:
            print(f"  {ecu}  DTC={dtc_code}  Status=0x{status:02X}")
```

## 四、DTC 故障码解析规则 (ISO 15031-6)

DTC 由 3 字节组成:

```
字节0(高): 类别(高2位) + 系统(低6位)
  高2位: 00=Powertrain  01=Chassis  10=Body  11=Network
字节1(中): 子系统(高4位) + 故障类型高(低4位)
字节2(低): 故障类型低 + 具体故障
```

**解析示例:**
```
DTC 050102:
  字节0=0x05: 高2位=00→P码(动力), 低6位=05→车速/怠速控制
  字节1=0x01: 子系统=0, 故障类型高=1
  字节2=0x02: 故障=02
  → P0501 → 车速传感器范围/性能

DTC 071186:
  → P1186 → 燃油压力传感器电路

DTC 010010:
  → P0010 → 进气凸轮轴位置执行器电路(Bank1)
```

## 五、ECU 诊断ID 映射表 (来自整车CAN架构)

| 诊断ID | 应答ID | ECU名称 | 所属系统 |
|--------|--------|---------|---------|
| 0x704 | 0x784 | 发动机控制模块(ECM) | 动力 |
| 0x709 | 0x789 | 中央网关(GW) | 全部 |
| 0x70F | 0x77F | 变速箱控制(TCU) | 动力 |
| 0x716 | 0x786 | ABS/ESC制动控制 | 底盘 |
| 0x717 | 0x787 | 电动助力转向(EPS) | 底盘 |
| 0x718 | 0x788 | 组合仪表(IC) | 车身 |
| 0x71A | 0x78A | 车身控制模块(BCM) | 车身 |
| 0x71C | 0x78C | 空调系统(HVAC) | 车身 |
| 0x71D | 0x78D | 安全气囊(SRS) | 底盘 |
| 0x71E | 0x78E | 泊车辅助(PDC) | 车身 |
| 0x720 | 0x790 | 胎压监测(TPMS) | 车身 |
| 0x723 | 0x793 | 电池管理系统(BMS) | 动力 |
| 0x724 | 0x794 | 车载充电机(OBC) | 动力 |
| 0x72E | 0x79E | 整车控制器(VCU) | 动力 |
| 0x730 | 0x7A0 | ADAS域控制器 | 底盘 |
| 0x732 | 0x7A2 | 前向毫米波雷达 | 底盘 |
| 0x733 | 0x7A3 | 前视智能摄像头 | 底盘 |
| 0x734 | 0x7A4 | 角雷达(左前) | 底盘 |
| 0x735 | 0x7A5 | 角雷达(右前) | 底盘 |
| 0x736 | 0x7A6 | 角雷达(左后) | 底盘 |
| 0x737 | 0x7A7 | 角雷达(右后) | 底盘 |
| 0x73A | 0x7AA | 驾驶员监测(DMS) | 车身 |
| 0x73B | 0x7AB | 环视影像系统 | 车身 |
| 0x73D | 0x7AD | OTA升级模块 | 信息 |
| 0x73E | 0x7AE | 车载T-BOX | 信息 |
| 0x741 | 0x7B1 | 左前门模块 | 车身 |
| 0x742 | 0x7B2 | 右前门模块 | 车身 |
| 0x743 | 0x7B3 | 左后门模块 | 车身 |
| 0x744 | 0x7B4 | 右后门模块 | 车身 |
| 0x745 | 0x7B5 | 电动尾门模块 | 车身 |
| 0x746 | 0x7B6 | 天窗控制模块 | 车身 |
| 0x747 | 0x7B7 | 座椅控制模块 | 车身 |
| 0x750 | 0x7C0 | 灯光控制模块 | 车身 |
| 0x755 | 0x7C5 | 雨量/光照传感器 | 车身 |
| 0x758 | 0x7C8 | 电子驻车(EPB) | 底盘 |
| 0x75A | 0x7CA | 电子悬架(CDC) | 底盘 |
| 0x75B | 0x7CB | 转向角传感器(SAS) | 底盘 |
| 0x75C | 0x7CC | 横摆率传感器(YRS) | 底盘 |
| 0x75D | 0x7CD | 加速度传感器 | 底盘 |
| 0x761 | 0x769 | DC-DC转换器 | 动力 |
| 0x775 | 0x77D | 电机控制器(MCU前) | 动力 |
| 0x776 | 0x77E | 电机控制器(MCU后) | 动力 |
| 0x777 | 0x77F | 电机控制器(MCU) | 动力 |
| 0x7E2 | 0x789 | PTC电加热器 | 动力 |
| 0x7E3 | 0x7EB | 电动压缩机(EAC) | 动力 |
| 0x7E4 | 0x7EC | 热管理模块(TMM) | 动力 |
| 0x7E5 | 0x7ED | 高压配电盒(PDU) | 动力 |
| 0x7E6 | 0x7EE | 绝缘监测(IMD) | 动力 |
| 0x7F1 | 0x7F9 | 诊断测试器 | 全部 |

## 六、分系统诊断方法

### 6.1 动力系统 (发动机/变速箱/混动)

**涉及ECU**: 0x704, 0x70F, 0x723, 0x775-777

**常读DID**: 发动机转速(F4 05), 冷却液温(F4 0C), 车速(F4 0D), 油门踏板(F4 2F), 环境温度(F4 46)

**诊断流程**:
1. 19 0A 读取DTC → 记录所有P/C/U码
2. 19 04 读取冻结帧 → 获取故障发生时的工况(转速/温度/车速/负荷)
3. F4 05 检查发动机转速传感器 → 怠速正常750±50rpm
4. F4 0C 检查冷却液温度 → 热车85-105°C
5. F4 0F 检查进气温度 → 与环境温度差<20°C
6. F4 11 检查节气门位置 → 怠速5-15%
7. F4 14 检查氧传感器 → 0.1-0.9V波动
8. F4 23 检查燃油压力 → 3-5bar(直喷更高)
9. F4 3C-43 检查失火计数 → 各缸接近0
10. F4 0E 检查点火提前角 → 怠速5-15°
11. F4 5A 检查VVT控制 → 实际角度应跟随目标
12. TCU读油温 → <120°C
13. TCU读输入/输出转速 → 速比匹配
14. 14 FF FF FF 清除DTC → 路试验证

### 6.2 底盘系统 (制动/转向/悬架)

**涉及ECU**: 0x716, 0x717, 0x758, 0x75A, 0x75B, 0x75C

**诊断流程**:
1. 19 0A 读取全系统DTC
2. 读取4轮轮速 → 车速20km/h四轮一致
3. SAS读转向角 → 正中=0°, 范围±540°
4. YRS读横摆率 → 静止≈0
5. 制动液压力传感器 → 踩制动时>50bar
6. EPB读电机电流 → 夹紧<20A, 释放<15A
7. CDC读悬架高度传感器 → 四角高度一致
8. SAS零点标定 → 直线行驶执行标定
9. ESC排气 → 更换制动液后执行

### 6.3 车身系统

**涉及ECU**: 0x71A, 0x718, 0x71C, 0x71E, 0x720, 0x741-747, 0x750, 0x755

**诊断流程**:
1. 19 0A 读取全系统DTC
2. BCM读供电电压 → 11-14.5V
3. 检查各门锁开关信号 → BCM DID
4. 灯光模块逐灯检查开路/短路
5. 车窗防夹标定 → RoutineControl 31服务
6. 天窗零位学习 → RoutineControl 31服务
7. 座椅记忆复位 → RoutineControl 31服务

### 6.4 网络通信

**涉及ECU**: 0x709(网关)

**诊断流程**:
1. 读网关DTC → 关注U0xxx通信丢失码
2. 看各网段总线负载 → 正常<50%, 峰值<70%
3. 断电测OBD 6-14终端电阻 → 标准60Ω±10Ω
4. 测CAN_H/CAN_L对地电阻 → >1MΩ
5. 测CAN_H对CAN_L电压 → 隐性≈0V, 显性≈2V
6. 示波器看波形 → 有无反射/干扰/电平偏移
7. 逐个断开ECU → 排查总线负载异常源

### 6.5 高压系统 (新能源)

**涉及ECU**: 0x723, 0x724, 0x72E, 0x761, 0x775-777, 0x7E2-E6

**诊断流程**:
1. 19 0A 全系统DTC
2. BMS: SOC, SOH, 总压, 单体压差<50mV, 温度<45°C
3. IMD读绝缘电阻 → >500Ω/V (如400V车型>200kΩ)
4. 检查高压互锁回路 → 导通
5. MCU读电机温度 <150°C, IGBT温度 <85°C
6. OBC读充电电压/电流 → 匹配铭牌
7. DCDC读低压输出 → 13.5-14.5V
8. PTC读工作电流 → 匹配额定功率
9. 高压断电等5分钟再操作（电容放电）
10. 上电时序验证 → VCU控制流程

### 6.6 ADAS系统

**涉及ECU**: 0x730, 0x732-737, 0x73A, 0x73B

**诊断流程**:
1. 19 0A DTC读取
2. 雷达校准状态检查 → DID读取偏角
3. 摄像头校准状态 → DID读取标定状态
4. 雷达阻挡检测 → 检查是否有遮挡物/污物
5. 传感器供电电压 → 各雷达/摄像头独立供电
6. ADAS域控通信 → 检查各传感器CAN通信状态

## 七、DTC 状态字节解读

19服务返回的DTC状态(1字节8位):

| Bit | 含义 | 解释 |
|-----|------|------|
| 0 | testFailed | 本次检测故障 |
| 1 | testFailedThisOpCycle | 本操作周期内故障 |
| 2 | pendingDTC | 待确认DTC |
| 3 | confirmedDTC | 已确认DTC(须维修) |
| 4 | testNotCompleteSinceClear | 清除后未完成检测 |
| 5 | testFailedSinceClear | 清除后曾故障 |
| 6 | testNotCompleteThisCycle | 本周期未完成检测 |
| 7 | warningIndicatorRequested | 已请求亮警告灯 |

**常见值:**
- `0x2F` (bit 0,1,2,3,5) — 已确认 + 待确认 + 曾故障 + 亮灯 → **立即维修**
- `0x0B` (bit 0,1,3) — 已确认 + 当前故障 → **故障持续**
- `0x08` (bit 3) — 仅已确认 → **历史故障，可能已自愈**

## 八、诊断报告模板

```
===========================================================
                    车辆故障诊断报告
===========================================================
车辆: VIN=____  车型=____  里程=____km  诊断日期=____
工具: ____

一、DTC汇总
序号  ECU           诊断ID   DTC码    状态   故障描述
----  -------------  ------  --------  ----   ----------
1     ECM发动机     0x704   050102   2F     车速传感器范围/性能
2     BMS电池       0x723   010010   08     电池单体电压低

二、系统分类诊断
【动力系统】
  DTC P0501(2F): 车速传感器范围/性能
    冻结帧: 转速=2000rpm, 车速=0km/h, 档位=D
    诊断: 读4轮轮速→右前=0, 其余正常
    判定: 右前轮速传感器故障
    建议: 更换右前轮速传感器, 清除DTC后路试验证

【高压系统】
  DTC P0010(08): 电池单体电压低
    诊断: 读BMS单体电压→#12单体=2.8V, 其余3.2V
    判定: #12单体欠压, 需进一步检查电池包
    建议: 拆检电池包, 测量#12单体内阻和容量

三、维修优先级
[高] 更换右前轮速传感器 → 恢复ABS/ESC
[中] 检查电池包#12单体 → 可能需均衡或更换模组

诊断技师: ____    日期: ____
===========================================================
```

## 九、安全注意事项

1. **高压安全**: 戴绝缘手套/护目镜, 高压断电后等5分钟(电容放电)
2. **气囊安全**: SRS诊断前断开蓄电池等1分钟
3. **制动安全**: ESC排气后在安全场地验证制动效果
4. **ADAS标定**: 更换雷达/摄像头/前挡风玻璃后必须重新标定
5. **不随意清码**: 确认故障已修复再清, 否则丢失诊断线索
6. **DTC不直接等于换件**: DTC指向的是"电路/传感器信号异常", 需信号+波形+数据流综合判断, 不排除线束/接插件问题