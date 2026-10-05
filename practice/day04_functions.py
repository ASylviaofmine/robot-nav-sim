#Mission 1:
def say_hello():
    print("我在运行")

say_hello()
print()

#Mission 2:
def say_hello(name):
    print(f"你好，{name}")

say_hello("小李")
say_hello("小王")
print()

#Mission 3:
def double(x):
    return x * 2

print(double(5))     # 输出 10
print()

#Mission 4: 造答案的机器（出货口交出一个 1~100 的随机数）
import random

def make_answer():
    return random.randint(1, 100)

#Mission 5: 问猜数的机器（投料口是提示语，出货口交出整数）
def ask_guess():
    return int(input("猜一个 1~100 的数："))

#Mission 6: 裁判机器（两个投料口，出货口交出判词）
def judge(guess, answer):
    if guess > answer:
        return "bigger"
    elif guess < answer:
        return "smaller"
    else:
        return "correct"

#Mission 7: 主流程——把三台机器串成完整的游戏
def play():
    answer = make_answer()
    count = 0
    while True:
        guess = ask_guess()
        count = count + 1
        hint = judge(guess, answer)
        print(hint)
        if hint == "correct":      # 暗号必须和 judge 里的完全一致
            print(f"你一共猜了 {count} 次")
            break

play()                             # 按下总开关，游戏开始
