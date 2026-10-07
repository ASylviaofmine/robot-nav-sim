#Mission 1
row = ["#", ".", "."]
print(row[0])      # 输出 #
print(row[-1])     # 输出 .
print(len(row))    # 输出 3
row.append("#")    # 往末尾再塞一个
print(row)         # 输出 ['#', '.', '.', '#']


#Mission 2
grid = [
    ["#", "#", "#"],
    ["#", ".", "#"],
    ["#", ".", "#"],
]
print(grid[1])      # 输出 ['#', '.', '#']  ← 第 2 行（编号从 0 开始！）
print(grid[1][1])   # 输出 .                ← 第 2 行第 2 个格子
print()
print()


#Mission3：
for row in grid:
    print(row)              # 输出 ['#', '#', '#'] 带括号引号，很丑

print("---")

for row in grid:
    print("".join(row))     # 输出 ###  ← 这才是地图该有的样子


#Mission 4: 5×5 地图
map5 = [
    ["#", "#", "#", "#", "#"],      # ← 第 0 行：全墙
    ["#", ".", ".", ".", "#"],      # ← 第 1 行：两边墙，中间空地
    ["#", ".", ".", ".", "#"],
    ["#", ".", ".", ".", "#"],      # 这里照规律补第 3、4 行
    ["#", "#", "#", "#", "#"],
]

for row in map5:
    print("".join(row))
print()

#Mission 5: 自动生成 10×10 地图
print("Mission 5:")
size = 10
map10 = []

for i in range(size):                    # 外层：i 是行号，0~9
    row = []                             # 每次先造一个空行
    for j in range(size):                # 内层：j 是列号，0~9
        if i == 0 or i == size - 1 or j == 0 or j == size - 1:
            row.append("#")              # 站在最外圈 → 是墙
        else:
            row.append(".")           # 空 1：不在最外圈，该放什么？

    map10.append(row)                 # 空 2：把造好的这一行放进大地图

for row in map10:
    print("".join(row))


#Mission 6: 放置起点和终点
print("Mission 6:")
for row in map10:
    print("".join(row))
print()
start = {"row": 1, "col": 1}
goal  = {"row": 8, "col": 8}

map10[start["row"]][start["col"]] = "S"   # 空 1：两个空都是同一个名字
map10[goal["row"]][goal["col"]] = "T"    # 空 2：该放哪个字母？

for row in map10:
    print("".join(row))
