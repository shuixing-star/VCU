MX_CAN-SV3.03-1 通讯协议

1、测试注意要点....1
2、通信协议....2
2.1 CAN 数据位....2
2.2 驱动器接收（驱动器 CAN 默认参数）....2
2.3 驱动器发送（扩展数据帧）....3
2.4 上位机 CAN 发送测试码（hex）....3
故障代码表....4

## 1、测试注意要点

◆ 多字节通信，默认高字节在前，低字节在后。

◆ CAN默认超时 1.5 秒驱动器没有接收到数据，电机停止。

◆ 标注“选用”（用户如需要此功能，请提前说明）

## 2、通信协议

## 2.1 CAN 数据位

<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>数据 0</td><td>数据 1</td><td>数据 2</td><td>数据 3</td><td>数据 4</td><td>数据 5</td><td>数据 6</td><td>数据 7</td></tr></table>

## 2.2驱动器接收（驱动器CAN 默认参数）

<table><tr><td>ID(扩展帧测试)</td><td>测试间隔时间</td><td>数据长度</td><td>波特率</td></tr><tr><td>1801E600</td><td>100ms</td><td>8byte</td><td>500k</td></tr></table>

<table><tr><td>字节</td><td>指令名称</td><td>数据类型与范围</td><td>单位</td><td>备注</td></tr><tr><td>Data0</td><td></td><td>Uint8(0~255)</td><td></td><td>Bit7 0:轴1扭矩模式1:轴1速度模式bit6 0:轴2扭矩模式1:轴2速度模式bit4,bit5 00:备用bit3:0:滑行失能(选用)1:滑行使能bit2:0:复位失能(选用)1:复位使能bit1,bit0:使能位00:轴2和轴1同时失能01:轴2使能,轴1失能10:轴2失能,轴1使能11:轴2和轴1同时使能</td></tr><tr><td>Data1</td><td>轴2“转速”或“扭矩”高字节</td><td rowspan="2">int16(-30000~30000)</td><td rowspan="2">r/min(A)</td><td rowspan="2">正值:电机正转零:电机停止负值:电机反转</td></tr><tr><td>Data2</td><td>轴2“转速”或“扭矩”低字节</td></tr><tr><td>Data3</td><td>轴2加/减速度</td><td>Uint8(0~255)</td><td></td><td>速度模式:每秒转速变化量(单位:10转)。如:加/减速为100,则转速每秒变化1000转。扭矩模式:每秒电流变化量(单位:A)</td></tr><tr><td>Data4</td><td>轴1“转速”或“扭矩”高字节</td><td rowspan="2">int16(-30000~30000)</td><td rowspan="2">r/min(A)</td><td rowspan="2">正值:电机正转零:电机停止负值:电机反转</td></tr><tr><td>Data5</td><td>轴1“转速”或“扭矩”低字节</td></tr><tr><td>Data6</td><td>轴1加/减速度</td><td>Uint8(0~255)</td><td></td><td>速度模式:每秒转速变化量(单位:10转)。如:加/减速为100,则转速每秒变化1000转。扭矩模式:每秒电流变化量(单位:A)</td></tr><tr><td>Data7</td><td>备用</td><td></td><td></td><td>默认值:0</td></tr></table>


2.3驱动器发送（扩展数据帧）


<table><tr><td>ID(扩展帧测试)</td><td>测试间隔时间</td><td>数据长度</td><td>波特率</td></tr><tr><td>1801E001</td><td>100ms</td><td>8byte</td><td>500k</td></tr></table>

<table><tr><td>字节</td><td>指令名称</td><td>数据类型</td><td>单位</td><td>备注</td></tr><tr><td>Data0</td><td>故障信息</td><td>Uint8</td><td></td><td rowspan="2">故障代码表</td></tr><tr><td>Data1</td><td>驱动器温度</td><td>Uint8</td><td>°C</td></tr><tr><td>Data2</td><td>轴2转速高字节</td><td rowspan="2">Int16(-32768~32767)</td><td rowspan="4">r/min</td><td rowspan="4">正值:电机正转零:电机停止负值:电机反转</td></tr><tr><td>Data3</td><td>轴2转速低字节</td></tr><tr><td>Data4</td><td>轴1转速高字节</td><td rowspan="2">Int16(-32768~32767)</td></tr><tr><td>Data5</td><td>轴1转速低字节</td></tr><tr><td>Data6</td><td>电源电压高字节</td><td rowspan="2">Uint16</td><td rowspan="2">0.1V</td><td rowspan="2"></td></tr><tr><td>Data7</td><td>电源电压低字节</td></tr></table>

## 2.4 上位机CAN发送测试码（hex）


Motro1和 Motor2 同时使能并且工作在“速度模式”


<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>C3</td><td>00</td><td>32</td><td>64</td><td>00</td><td>32</td><td>64</td><td>00</td></tr></table>


Motro1使能，Motor2失能并且工作在“速度模式”


<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>C2</td><td>00</td><td>32</td><td>64</td><td>00</td><td>32</td><td>64</td><td>00</td></tr></table>


Motro1失能，Motor2使能并且工作在“速度模式”


<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>C1</td><td>00</td><td>32</td><td>64</td><td>00</td><td>32</td><td>64</td><td>00</td></tr></table>


Motro1和 Motor2 同时使能并且工作在“扭矩模式”


<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>03</td><td>00</td><td>14</td><td>64</td><td>00</td><td>14</td><td>64</td><td>00</td></tr></table>


Motro1使能，Motor2失能并且工作在“扭矩模式”


<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>02</td><td>00</td><td>14</td><td>64</td><td>00</td><td>14</td><td>64</td><td>00</td></tr></table>


Motro1失能，Motor2使能并且工作在“扭矩模式”


<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>01</td><td>00</td><td>14</td><td>64</td><td>00</td><td>14</td><td>64</td><td>00</td></tr></table>

复位

<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>04</td><td>00</td><td>00</td><td>64</td><td>00</td><td>00</td><td>64</td><td>00</td></tr></table>

滑行

<table><tr><td>Data0</td><td>Data1</td><td>Data2</td><td>Data3</td><td>Data4</td><td>Data5</td><td>Data6</td><td>Data7</td></tr><tr><td>08</td><td>00</td><td>00</td><td>64</td><td>00</td><td>00</td><td>64</td><td>00</td></tr></table>

故障代码表

<table><tr><td>故障代码(十进制)</td><td></td></tr><tr><td>0</td><td>正常</td></tr><tr><td>1</td><td rowspan="4">驱动器故障</td></tr><tr><td>2</td></tr><tr><td>3</td></tr><tr><td>4</td></tr><tr><td>5</td><td>过流</td></tr><tr><td>6</td><td>过压</td></tr><tr><td>7</td><td>欠压</td></tr><tr><td>8</td><td>过温</td></tr><tr><td>9</td><td></td></tr><tr><td>10</td><td></td></tr><tr><td>11</td><td>电机2超速</td></tr><tr><td>12</td><td>电机1超速</td></tr><tr><td>13</td><td>电机2过载</td></tr><tr><td>14</td><td>电机1过载</td></tr><tr><td>15</td><td>电机2缺相</td></tr><tr><td>16</td><td>电机1缺相</td></tr><tr><td>17</td><td>电机2抱闸</td></tr><tr><td>18</td><td>电机1抱闸</td></tr><tr><td>19</td><td>电机2编码器</td></tr><tr><td>20</td><td>电机1编码器</td></tr><tr><td>21</td><td>电机2过温</td></tr><tr><td>22</td><td>电机1过温</td></tr><tr><td>23</td><td>电机2霍尔故障</td></tr><tr><td>24</td><td>电机1霍尔故障</td></tr><tr><td>25</td><td>电机2堵转</td></tr><tr><td>26</td><td>电机1堵转</td></tr><tr><td>27</td><td>UART通讯故障</td></tr><tr><td>28</td><td>RS485通讯故障</td></tr><tr><td>29</td><td>CAN通讯故障</td></tr><tr><td>30</td><td>摇杆故障</td></tr><tr><td>31</td><td>转把故障</td></tr></table>