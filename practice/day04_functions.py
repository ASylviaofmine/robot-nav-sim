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

#Mission 4:
import random

def make_answer():
    return random.randint(1, 1000)

print(make_answer())


#Mission 5：
def ask_guess():
        return int(input("猜一个 1~100 的数："))

print(ask_guess())

#Mission 6：
def judge(guess, answer):
    if guess > answer:
        return "bigger"
    elif guess < answer:
        return "smaller"
    else:
        return "correct"
print(judge(33, 50))
print(judge(20, 50))
print(judge(50, 50))

