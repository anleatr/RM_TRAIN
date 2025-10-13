import os
import sys
import time
import pandas as pd
import torch
import torch.nn as nn
from PIL import Image
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms

# 转换成28*28的Tensor格式
# trans = transforms.Compose([
#     transforms.Resize((28, 28)),
#     transforms.ToTensor(),
# ])
trans = transforms.ToTensor()


def default_loader(path):
    """
    定义读取图片的格式为28*28的单通道灰度图
    :param path: 图片路径
    :return: 图片
    """
    return Image.open(path).convert('L').resize((28, 28))


class MyDataset(Dataset):
    """
    制作数据集
    """

    def __init__(self, csv_path, transform=None, loader=default_loader):
        """
        :param csv_path: 文件路径
        :param transform: 转后后的Tensor格式
        :param loader: 图片加载方式
        """
        super(MyDataset, self).__init__()
        df = pd.read_csv(csv_path, engine="python", encoding="utf-8")
        self.df = df
        self.transform = transform
        self.loader = loader

    def __getitem__(self, index):
        """
        按照索引从数据集提取对应样本的信息
        Args:
            index: 索引值
        Returns:
            特征和标签
        """
        fn = self.df.iloc[index][0]
        label = self.df.iloc[index][1]
        img = self.loader(fn)
        # 按照路径读取图片
        if self.transform is not None:
            # 数据标签转换为Tensor
            img = self.transform(img)

        return img, label

    def __len__(self):
        """
        样本数量
        Returns:
            样本数量
        """
        return len(self.df)


class LeNet(nn.Module):
    """
    定义模型, 这里使用LeNet
    """

    def __init__(self):
        super(LeNet, self).__init__()
        # 卷积层
        self.conv = nn.Sequential(
            # 输入通道数, 输出通道数, kernel_size
            nn.Conv2d(1, 6, 5),
            nn.Sigmoid(),
            # 最大池化
            nn.MaxPool2d(2, 2),
            nn.Conv2d(6, 16, 5),
            nn.Sigmoid(),
            nn.MaxPool2d(2, 2)
        )
        # 全连接层
        self.fc = nn.Sequential(
            nn.Linear(16 * 4 * 4, 120),
            nn.Sigmoid(),
            nn.Linear(120, 84),
            nn.Sigmoid(),
            nn.Linear(84, 9) #种类
        )

    def forward(self, img):
        feature = self.conv(img)
        output = self.fc(feature.reshape(img.shape[0], -1))
        return output


# %% 模型评估
def evaluate_accuracy(data_iter, net, device=None):
    """
    评估模型, GPU加速运算
    :param data_iter: 测试集迭代器
    :param net: 待评估模型
    :param device: 训练设备
    :return: 正确率
    """

    # 未指定训练设备的话就使用 net 的 device
    if device is None and isinstance(net, nn.Module):
        device = list(net.parameters())[0].device
    acc_sum, n = 0.0, 0
    with torch.no_grad():
        for x, y in data_iter:
            if isinstance(net, nn.Module):
                # 评估模式, 关闭dropout(丢弃法)
                net.eval()
                acc_sum += (net(x.to(device)).argmax(dim=1) == y.to(device)).float().sum().cpu().item()
                # 改回训练模式
                net.train()
            n += y.shape[0]
    return acc_sum / n


def train_model(net, train_iter, test_iter, loss_func, optimizer, device, num_epochs):
    """
    训练模型
    :param net: 原始网络
    :param train_iter: 训练集
    :param test_iter: 测试集
    :param loss_func: 损失函数
    :param optimizer: 优化器
    :param device: 训练设备
    :param num_epochs: 训练周期
    :return: 无
    """

    net = net.to(device)
    print("训练设备={0}".format(device))
    for i in range(num_epochs):
        # 总误差, 准确率
        train_lose_sum, train_acc_sum = 0.0, 0.0
        # 样本数量, 批次数量
        sample_count, batch_count = 0, 0
        # 训练时间
        start = time.time()
        for x, y in train_iter:
            # x, y = j
            x = x.to(device)
            y = y.long().to(device)
            y_output = net(x)
            lose = loss_func(y_output, y)
            optimizer.zero_grad()
            lose.backward()
            optimizer.step()
            train_lose_sum += lose.cpu().item()
            train_acc_sum += (y_output.argmax(dim=1) == y).sum().cpu().item()
            sample_count += y.shape[0]
            batch_count += 1
        test_acc = evaluate_accuracy(test_iter, net)
        print("第{0}个周期, lose={1:.3f}, train_acc={2:.3f}, test_acc={3:.3f}, time={4:.1f}".format(
            i, train_lose_sum / batch_count, train_acc_sum / sample_count, test_acc, time.time() - start
        ))


if __name__ == '__main__':

    # 数据集元数据文件的路径
    metadata_path = 'datasets/label.csv'

    # 训练设备
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # device = "cpu"
    # 批次规模
    batch_size = 8

    # 线程数
    if sys.platform == "win32":
        num_workers = 0
    else:
        num_workers = 12

    # 训练集占比
    train_rate = 0.8

    # 创建数据集
    # src_data = MyDataset(csv_path=metadata_path, transform=trans)
    src_data = MyDataset(csv_path=metadata_path, transform=trans)

    print('num_of_trainData:', len(src_data))

    # for i, j in train_iter:
    #     print(i.size(), j.size())
    #     break

    net = LeNet()
    print(net)

    # 训练次数
    num_epochs = 5

    # 优化算法
    optimizer = torch.optim.Adam(net.parameters(), lr=0.002)

    # 交叉熵损失函数
    loss_func = nn.CrossEntropyLoss()

    for i in range(10):
        # K折交叉验证
        train_size = int(train_rate * len(src_data))
        test_size = len(src_data) - train_size
        train_set, test_set = torch.utils.data.random_split(src_data, [train_size, test_size])
        train_iter = DataLoader(dataset=train_set, batch_size=batch_size, shuffle=True, num_workers=num_workers)
        test_iter = DataLoader(dataset=test_set, batch_size=batch_size, shuffle=True, num_workers=num_workers)
        train_model(net, train_iter, test_iter, loss_func, optimizer, device, num_epochs)

    # 保存训练后的模型数据
    torch.save(net.state_dict(), "./model_param/state_dict.pt")
    
