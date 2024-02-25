# DFS路径搜索

难点分析：

1. 一个是路径回溯。
2. 一个是把具体问题转为抽象的栅格地图。

引申出新的问题：（性能优化很重要）

1、如何查看py运行的内存占用和cpu占用。

2、如何查看cpp运行的内存占用和cpu占用。

```python
#python伪代码实现，使用非递归的算法实现
snode = init_snode()
gnode = init_gnode() # 设置起点和终点
close_set = dict()
open_set = dict() # 初始化两个容器，这里的字典是有序字典, open_set作为一个栈来使用
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
# 回溯查找，得到路径
path = search_back(gnode,close_set)
```

