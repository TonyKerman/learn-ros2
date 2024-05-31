# BehaviorTree cpp v3 内置节点介绍

## SetBlackboard
定义/修改一个 `blackboard`键值对
### Input
* value : 要定义/修改的键值对的值
* output_key : 键值对的键
### eg
```xml
<SetBlackboard output_key="rgoal" value="1,-1.0;2,1.0" />
```
```xml
<SetBlackboard output_key="goal" value="0.0;map;9.5;2.0;0.0;0.0;0.0;0.0;1.0"/>
```

## BlackboardCheckInt

## SwitchNode
SwitchNode 相当于一个switch语句,根据传入的值执行对应子分支.
SwitchNode 必须有n+1个分支,最后一个分支为默认分支,当值全部不匹配时执行默认分支
### Input
* variable :要被判断的值
* case1,case2.... : 当variable等于此值时,执行其分支,如果都不匹配,执行默认分支


### eg:

```xml
<Switch3 variable="{var}" case_1="1" case_2="42" case_3="666"> <ActionA name="action_when_var_eq_1"> <ActionB name="action_when_var_eq_42"> <ActionC name="action_when_var_eq_666"> <ActionD name="default_action"> </Switch3>
```
