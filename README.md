### 基于分布式架构的聊天机器人

#### 需求分析

```
服务节点：
节点注册服务、节点查询服务、节点清理服务。

聊天节点：
节点名称启动配置
服务调用支持
topic接收消息
topic发布消息
心跳维护功能
terminal界面支持，包含查询命令、退出命令、输入发送消息、@私聊功能。
```

#### 架构设计

```

服务发现与注册：节点如何找到彼此

消息传递模式：发布/订阅 vs 请求/响应

负载均衡：如何在节点间分配工作

容错性：节点故障时的处理

可扩展性：轻松添加更多机器人节点
```

#### 模块实现

**聊天节点**

```
成员函数
行为函数：构造请求，利用客户端调用注册
行为函数：构造请求，利用客户端调用获取
ROS函数：发表注册方法，心跳方法，定时注册服务，更新存活时间。
ROS函数：订阅注册方法，接收消息并处理。

线程启动函数：启动输入线程。
函数流程：注册服务（成功or失败）--启动线程（deatch模式)--ROSspin循环启动

输入线程函数：循环从窗口捕获输入，正常广播发送，特殊私聊发送。

成员变量
节点名称：string
节点id：string
运行状态：bool atomic
输入线程：thread

订阅注册：Subscription
发表注册：Publisher
注册服务客户端：Client
获取节点客户端：Client
心跳定时器：Timer
```

**发现服务模块**

```
成员函数
处理注册请求：HandleRegisterNode	
处理获取所有节点请求：HandleGetNodes

定时清理超时节点：CheckNodes

成员变量
存储节点的结构：unordered_map
并行控制：mutex
清理定时器：Timer
```

#### 测试结果

**节点登陆测试**

![A节点登陆](https://github.com/HelloDVA/ROS-2/blob/main/test1-2.png)

![B节点登陆](https://github.com/HelloDVA/ROS-2/blob/main/test1-3.png)

![服务节点](https://github.com/HelloDVA/ROS-2/blob/main/test1-2.png)

**群聊消息测试**

![A节点发送群聊消息](https://github.com/HelloDVA/ROS-2/blob/main/test2-1.png)

![B节点接收](https://github.com/HelloDVA/ROS-2/blob/main/test2-2.png)

**私发消息测试**

![B节点发送私聊消息](https://github.com/HelloDVA/ROS-2/blob/main/test3-1.png)

![A节点接收](https://github.com/HelloDVA/ROS-2/blob/main/test3-2.png)

**节点退出测试**

![A节点退出](https://github.com/HelloDVA/ROS-2/blob/main/test4-1.png)

![B节点退出](https://github.com/HelloDVA/ROS-2/blob/main/test4-2.png)

![服务节点清理失效节点](https://github.com/HelloDVA/ROS-2/blob/main/test4-3.png)
