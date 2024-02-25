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
python ./my_dfs.py
```

## 学习总结

其实思想非常简单，写代码难的地方在于需要配置的环境太复杂了，因此写算法抽象思维很重要，才能抓住精髓。

该算法思想重点在于，使用python的dict作为栈，有点像树的层序遍历，每遍历一个节点，就把该节点的邻居加入close_set，并设置父子级关系。直到找到目标点为止。

python代码写算法学到的点：

1. 使用dict作有序字典，充当栈使用
2. 作为启发，每次加入open_set的node，按照一定规则从close_set里面挑选，就实现了A star。算法思想是多么的朴实，因此基础很重要，把这些基础代码都熟练掌握了，排列组合式的创新也就手到擒来。
3. 考虑与cpp实现作为对比，open_set直接使用栈或者单链表，而close_set依旧使用字典。
