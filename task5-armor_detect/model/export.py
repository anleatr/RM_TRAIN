import torch
from train import LeNet                 # 1. 你的模型定义

model = LeNet()                       # 2. 实例化
model.load_state_dict(torch.load('model_param/state_dict.pt', map_location='cpu'))
model.eval()                        # 3. 灌权重

x = torch.randn(1, 1, 28, 28)     # 4. 假输入
ts = torch.jit.trace(model, x)     # 5. 生成图
ts.save('model_ts.pt')             # 6. 给 C++ 用