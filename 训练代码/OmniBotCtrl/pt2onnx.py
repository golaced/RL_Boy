import torch
import torch.onnx
#python -m onnxsim /home/pi/Downloads/unitree/humanoid-gym-main/model_jitt/policy_origin.onnx /home/pi/Downloads/unitree/humanoid-gym-main/model_jitt/policy_simple.onnx
# 定义输入尺寸
inputSize = 615  # 假设 length 是你定义的输入尺

dummy_input = torch.randn(1, inputSize, dtype=torch.float32)  # 创建虚拟输入张量
# 加载 TorchScript 模型
model = torch.jit.load("./model_jitt/policy_origin.pt")
# 将模型设置为推理模式
model.eval()
# 定义 ONNX 文件路径
onnx_file_path = "./model_jitt/policy_origin.onnx"
# 注册自定义操作（如果有）
# torch.onnx.register_custom_op_symbolic("::custom_op_name", custom_op_symbolic, 11)

# 导出模型为 ONNX 格式
torch.onnx.export(
    model,  # 模型
    dummy_input,  # 输入张量
    onnx_file_path,  # 输出文件路径
    export_params=True,  # 导出模型参数
    opset_version=13,  # ONNX 版本
    input_names=['input0'],  # 输入名称
    output_names=['output0'],  # 输出名称
    dynamic_axes={'input0': {0: 'batch_size'}, 'output0': {0: 'batch_size'}},  # 动态轴
    verbose=True  # 启用调试日志
)

print(f"模型已成功导出为 ONNX 格式，保存路径为：{onnx_file_path}")

from onnxruntime.quantization import quantize_dynamic, QuantType
onnx_file_path_quan= "./model_jitt/policy_quan_int8.onnx"
quantize_dynamic(onnx_file_path,onnx_file_path_quan, weight_type=QuantType.QInt8)
print(f"模型已成功量化 ONNX-QInt8 格式，保存路径为：{onnx_file_path_quan}")

# pip install onnx onnxconverter-common -i https://pypi.tuna.tsinghua.edu.cn/simple
import onnx
from onnxconverter_common import float16
# 加载原始 ONNX 模型
model = onnx.load(onnx_file_path)
# 将模型转换为 float16
model_fp16 = float16.convert_float_to_float16(model)
# 保存转换后的模型
onnx_file_path_quan16= "./model_jitt/policy_quan_fp16.onnx"
onnx.save(model_fp16, onnx_file_path_quan16)
print(f"模型已成功量化 ONNX-Float16 格式，保存路径为：{onnx_file_path_quan16}")
 
from optimum.onnxruntime import ORTOptimizer, OptimizationConfig
# 加载 ONNX 模型 pip install optimum -i https://pypi.tuna.tsinghua.edu.cn/simple
model_path = onnx_file_path
save_dir = "./model_jitt/policy_opt.onnx"
# 创建优化器
optimizer = ORTOptimizer.from_pretrained(model_path)
# optimization_level=0：禁用所有优化。
# optimization_level=1：启用基本优化，例如常量折叠。
# optimization_level=2：启用基本和扩展优化，包括复杂的节点融合。
# optimization_level=99：启用所有可用优化，包括布局优化。
# 定义优化配置
optimization_config = OptimizationConfig(
    optimization_level=2,  # 选择优化级别
    enable_transformers_specific_optimizations=True,  # 启用特定于 transformers 的优化
    optimize_for_gpu=False,  # 是否针对 GPU 进行优化
)

# 执行优化
optimizer.optimize(
    save_dir=save_dir,  # 保存优化后的模型路径
    optimization_config=optimization_config
)