# move_base笔记

1. move_base状态机中一共有三种状态，`PLANNING：正在规划路径`，`CONTROLLING：正在跟随路径`，`CLEARING：恢复动作`，首先得状态为PLANNING，规划到path后更改为CONTROLLING。
   1. 局部规划失败时间大于`controller_patience_` 时会触发恢复机制
   2. 全局规划失败时间大于`planner_patience_ `时会触发恢复机制
   3. 全局规划失败次数大于`max_planning_retries_`时会触发恢复机制
   4. 长时间困在一片小区域时间大于`oscillation_timeout_`时会触发恢复机制。注意当机器人移动`oscillation_distance_`后会重置`oscillation_timeout_`。
   5. `recovery_behavior_enabled_`：是否启用恢复机制来清理空间
