class NODE:
    def __init__(self, cost) -> None:
        self.cost = cost

    def __str__(self) -> str:
        return f"cost:{self.cost}"


open_set = dict()
open_set[1] = NODE(23)
open_set[2] = NODE(3)
open_set[3] = NODE(33)

# min函数妙用，妙不可言。用来处理字典
# 根据字典value求最小值
c_id = min(open_set, key=lambda o: open_set[o].cost)
print(c_id)
print(open_set[c_id])
# 根据k求最小值
v_id = min(open_set, key=lambda o: o)
print(v_id)
print(open_set[v_id])
