import os
import pandas as pd

root = 'datasets'                 # 你的根目录
save_csv = 'datasets/label.csv'   # 输出文件

# 1. 建立“文件夹名 -> 标签”映射
dir_to_label = {
    '1': 0,
    '2': 1,
    '3': 2,
    '4': 3,
    '5': 4,
    '6outpost': 5,
    '7guard': 6,
    '8base': 7,
    '9neg': 8,
}

records = []
for dirname, label in dir_to_label.items():
    folder_path = os.path.join(root, dirname)
    if not os.path.isdir(folder_path):
        continue
    for file in os.listdir(folder_path):
        # 只把常见图片后缀扫进来
        if file.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tif')):
            records.append({
                'image_path': os.path.join(folder_path, file),
                'label': label
            })

df = pd.DataFrame(records)
df.to_csv(save_csv, index=False)
print('Done! 共写入', len(df), '条记录 ->', save_csv)