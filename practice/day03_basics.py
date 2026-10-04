import random

answer = random.randint(1, 100)
count = 0

while True:
    guess = int(input("猜一个 1~100 的数："))
    count = count + 1

    if guess > answer:              # 空 1：跟谁比？比谁大算"大了"？
        print("大了")
    elif guess < answer:
        print("小了")
    else:
        print(f"猜对了！你一共猜了 {count} 次")# 空 2：怎么让循环停下来？
        break