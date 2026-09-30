#任务1
#温度转换器（今天的正式产出）
#输入一个摄氏温度 → 输出换算后的华氏温度，格式 25.0C = 77.0F（保留 1 位小数）。公式：F = C × 1.8 + 32

print("mission 1:")
c = float(input("请输入摄氏温度："))      # 假设输入 25
f = c * 1.8 + 32
print(f"{c} 摄氏度 = {f:.1f} 华氏度")     # 输出 25.0 摄氏度 = 77.0 华氏度

#任务2
#自我介绍卡
#输入姓名和年龄 → 输出 我叫 XX，今年 XX 岁，10 年后我 XX 岁（最后一处年龄必须算出来，不许手填）

print("mission 2:")
nam=input("请输入名字：")
age=input("请输入年龄：")
age2=int(age)+10
#一个 = 是存放，两个 == 是比较
print(f"我叫{nam}，今年{age}岁，10年后我{age2}岁。")

#任务3
#字符串解剖
#输入一个单词（比如 robot）→ 输出它的长度、第一个字母、最后一个字母、全大写形式

print("mission 3:")
word =input("请输入单词：")
print(len(word), word[0], word[-1], word.upper())
