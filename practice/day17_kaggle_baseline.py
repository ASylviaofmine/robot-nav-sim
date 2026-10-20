"""
D17 · Kaggle 首次提交
赛题：Playground S6E10 - Predicting Airline Satisfaction
评分：ROC-AUC（不是准确率！预测值要填概率）

运行方式（PyCharm 里直接 Run，或终端）：
    cd robot-nav-sim
    .venv/Scripts/python.exe practice/day17_kaggle_baseline.py

产出：robot-nav-sim/submission.csv（30 万行），上传到 Kaggle 的 Submit Prediction

────────────────────────────────────────────────────────
你要填 4 个空，每个空上面都标了 ① ② ③ ④ 和提示。
填完后先自己猜一下：这个 baseline 的 AUC 大约会在什么区间？
────────────────────────────────────────────────────────
"""

from pathlib import Path

import pandas as pd
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.model_selection import cross_val_score
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

BASE = Path(__file__).resolve().parent.parent
DATA = BASE / "data"
TARGET = "satisfaction"

train = pd.read_csv(DATA / "train.csv")
test = pd.read_csv(DATA / "test.csv")
print(f"训练集 {train.shape}  测试集 {test.shape}")
print(f"正例占比：{train[TARGET].mean():.3f}")

# ── 1. 把答案和特征分开 ─────────────────────────────────
# y 是"正确答案"，X 是"题目"（喂给模型的东西）
# 注意：satisfaction 这一列绝不能留在 X 里，否则就是抄答案
y = train[TARGET]                                                    # ①

X = train.drop(columns=[TARGET])
test_id = test["id"]          # id 不参与训练，但要留着写提交文件
X_test = test.drop(columns=["id"])

print(f"特征列数：{X.shape[1]}")

# ── 2. 类别列编码 ────────────────────────────────────────
# Gender / Customer Type / Type of Travel / Class 是文字，模型算不了加法
# one-hot：把 "Class" 变成 Class_Eco / Class_Business / Class_Eco Plus 三个 0-1 列
#
# 关键坑：训练集和测试集必须用同一套编码。
# 分开编码的话，测试集可能少一个 "Eco Plus" 列 → 列数对不上 → 模型直接报错
# 所以先把两份拼起来编码，再切回去
n_train = len(X)
both = pd.concat([X, X_test], ignore_index=True)

both = pd.get_dummies(both)                                                 # ②

X = both.iloc[:n_train]
X_test = both.iloc[n_train:]
print(f"编码后特征列数：{X.shape[1]}")

# ── 3. 缺失值 ────────────────────────────────────────────
# 只有 Arrival Delay in Minutes 缺 292 个（占 0.04%），用中位数填
# 中位数只看训练集，避免把测试集的信息漏进来
med = X.median(numeric_only=True)
X = X.fillna(med)
X_test = X_test.fillna(med)

# ── 4. 交叉验证 ──────────────────────────────────────────
# 把训练集切成 5 份，轮流用 4 份训练、1 份验证
# 这样做是为了拿到一个"没见过的数据上"的分数——这才是你真实水平的估计
model = HistGradientBoostingClassifier(max_iter=200, random_state=0)

scores = cross_val_score(model, X, y, cv=5, scoring="roc_auc")    # ③
print(f"CV AUC = {scores.mean():.4f} ± {scores.std():.4f}")

# ── 5. 全量训练 + 预测测试集 ──────────────────────────────
model.fit(X, y)

proba = model.predict_proba(X_test)   # shape = (300000, 2)：[不满意概率, 满意概率]
print(f"predict_proba 输出形状：{proba.shape}")

sub = pd.DataFrame({
    "id": test_id,
    "satisfaction": proba[:, 1]                                     # ④
})

out = BASE / "submission.csv"
sub.to_csv(out, index=False)

print(f"\n已写出 {out}")
print(f"行数 {len(sub)}  概率范围 {sub['satisfaction'].min():.4f} ~ {sub['satisfaction'].max():.4f}")
print(sub.head())

# ── 跑完之后 ─────────────────────────────────────────────
# 1. 打开 Kaggle 比赛页 → Submit Prediction → 上传 submission.csv
# 2. 记下 Public LB 的 AUC 和名次
# 3. 想一想：CV AUC 和 LB AUC 差多少？差得多说明什么？
