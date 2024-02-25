# Astar算法
该算法和DFS思想很像，就是多添加一个步骤，在open_set里面通过增加启发函数，计算open_set中代价最低的节点并挑选出来作为下一步。
需要注意的一点是，当根据启发函数走的路径遇到障碍物，路径折返时，可以更新临界点的parent，以保证路径回溯时选择更短的路径。如果不加也行，也是基本最优。

## 学习总结

```python
#min函数妙用，对字典的key和value求最小值
class NODE:
    def __init__(self, cost) -> None:
        self.cost = cost

    def __str__(self) -> str:
        return f"cost:{self.cost}"


open_set = dict()
open_set[1] = NODE(23)
open_set[2] = NODE(3)
open_set[3] = NODE(33)

c_id = min(open_set, key=lambda o: open_set[o].cost)
print(c_id) # 2
print(open_set[c_id]) # 3
# 根据k求最小值
v_id = min(open_set, key=lambda o: o)
print(v_id) # 1
print(open_set[v_id]) # 23
#详见test.py
```

