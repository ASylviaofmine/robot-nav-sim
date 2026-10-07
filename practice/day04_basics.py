import random

def make_answer():
    return random.randint(1, 100)

def ask_guess():
    return int(input("猜一个 1~100 的数："))

def judge(guess, answer):
    if guess>answer:                # 空 1
        return "bigger"
    elif guess<answer:              # 空 2
        return "smaller"
    else:
        return "correct"

def play():
    answer = make_answer()
    count = 0
    while True:
        guess = ask_guess()
        count = count + 1
        hint = judge(guess, answer)
        print(hint)
        if hint == "correct":
            print(f"你一共猜了 {count} 次")
            break

play()      # ← 最后这行必须在，不然什么都不会发生
