import matplotlib.pyplot as plt
import numpy as np

# 데이터 준비
categories = ['Category 1', 'Category 2', 'Category 3']
group1 = [5, 7, 3]
group2 = [6, 2, 4]
group3 = [4, 5, 6]

# X축 위치
x = np.arange(len(categories))

# 누적 바 그래프 그리기
plt.bar(x, group1, label='Group 1')
plt.bar(x, group2, bottom=group1, label='Group 2')
plt.bar(x, group3, bottom=np.array(group1) + np.array(group2), label='Group 3')

# 그래프 꾸미기
plt.xticks(x, categories)
plt.xlabel('Categories')
plt.ylabel('Values')
plt.title('Stacked Bar Chart Example')
plt.legend()

# 그래프 출력
plt.show()