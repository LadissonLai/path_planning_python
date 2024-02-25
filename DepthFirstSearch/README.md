# DFS深度优先搜索

要点分析：
1. 使用字典作为栈将递归算法转为非递归。
2. 邻居节点如何访问。
3. 搜索到目标点后，如何实现路径回溯。

```python
#python伪代码实现，使用非递归的算法实现
snode = init_snode()
gnode = init_gnode() # 设置起点和终点
close_set = dict()
open_set = dict() # 有序字典, open_set作为一个栈来使用。注：从py3.7以后，dict都是有序字典
open_set.add(snode)

while True:
    if open_set is empty：
    	raise Error("没有找到终点")
    	break
    current_node = open_set.pop()
    if current_node == goal_node:
        gnode.parent = current_node.parent
        break;
    for neighbor in current_node.neighbors:
        if neighbor not in close_set:
            close_set.add(neighbor);
            open_set.add(neighbor);
            neighbor.parent = current_node;
# 回溯查找，得到路径。类似于单链表的遍历。
path = search_back(gnode,close_set)
```

## Getting Started

```shell
conda activate ppp
python ./depth_first_search.py
```

