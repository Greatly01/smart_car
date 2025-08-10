# 能输出中文结果，但在txt里面乱码
# # Ultralytics YOLOv5 🚀, AGPL-3.0 license
# import argparse
# import os
# import time
# from pathlib import Path
# import torch
# import cv2
# from models.common import DetectMultiBackend
# from utils.general import (
#     LOGGER,
#     check_img_size,
#     non_max_suppression,
#     scale_boxes,
#     xyxy2xywh,
# )
# from utils.torch_utils import select_device
# from utils.dataloaders import LoadImages
# from utils.plots import colors

# # 假设这是中文类别名称映射，你可以根据实际情况修改
# CHINESE_CLASS_NAMES = {
#     'person': '人',
#     'bus': '公交车',
#     # 可以继续添加其他类别映射
# }


# def init_model(opt):
#     # 初始化模型
#     device = select_device(opt.device)
#     model = DetectMultiBackend(opt.weights, device=device, dnn=opt.dnn, data=opt.data, fp16=opt.half)
#     stride, names, pt = model.stride, model.names, model.pt
#     imgsz = check_img_size(opt.imgsz, s=stride)  # 检查图像尺寸
#     return model, device, stride, names, pt, imgsz


# @torch.no_grad()
# def main(opt):
#     # 提前初始化模型
#     model, device, stride, names, pt, imgsz = init_model(opt)

#     source_dir = Path(opt.source)
#     output_dir = Path(opt.output)
#     output_dir.mkdir(parents=True, exist_ok=True)  # 创建输出文件夹

#     processed_files = set()  # 记录已处理的文件
#     img_formats = [f".{f}" for f in ['bmp', 'dng', 'jpeg', 'jpg', 'mpo', 'png',
#                                       'tif', 'tiff', 'webp', 'pfm']]  # 支持的图像格式

#     LOGGER.info(f"开始监控文件夹: {source_dir}")
#     LOGGER.info(f"等待新图片存入，按Ctrl+C停止...")

#     try:
#         while True:
#             # 获取当前文件夹内所有符合格式的图片文件
#             current_files = [f for f in source_dir.glob('*')
#                              if f.suffix.lower() in img_formats and f.is_file()]

#             # 过滤未处理的新文件（按修改时间排序，确保新文件先处理）
#             new_files = [f for f in current_files if f.name not in processed_files]
#             new_files.sort(key=lambda x: x.stat().st_mtime)  # 按修改时间排序

#             for img_path in new_files:
#                 processed_files.add(img_path.name)  # 标记为已处理
#                 LOGGER.info(f"\n检测新图片: {img_path.name}")

#                 # 单图片处理流程
#                 dataset = LoadImages(str(img_path), img_size=imgsz, stride=stride, auto=pt)
#                 for path, im, im0s, _, _ in dataset:
#                     im = torch.from_numpy(im).to(device)
#                     im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
#                     im /= 255  # 0 - 255 to 0.0 - 1.0
#                     if len(im.shape) == 3:
#                         im = im[None]  # 扩展batch维度

#                     # 记录推理开始时间
#                     start_time = time.time()

#                     # 推理
#                     pred = model(im, augment=opt.augment, visualize=False)

#                     # 记录推理结束时间
#                     end_time = time.time()
#                     inference_time = end_time - start_time
#                     LOGGER.info(f"识别用时: {inference_time:.4f} 秒")

#                     # NMS
#                     pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
#                                                opt.classes, opt.agnostic_nms, opt.max_det)

#                     # 处理检测结果
#                     results = []
#                     im0 = im0s.copy()
#                     for det in pred:
#                         if len(det):
#                             det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

#                             # 收集检测结果
#                             for *xyxy, conf, cls in det:
#                                 c = int(cls)
#                                 original_name = names[c]
#                                 chinese_name = CHINESE_CLASS_NAMES.get(original_name, original_name)
#                                 label = f'{chinese_name} {conf:.2f}'
#                                 results.append((chinese_name, f"{conf:.2f}"))

#                                 # 手动绘制边界框和标签
#                                 xyxy = [int(i) for i in xyxy]
#                                 cv2.rectangle(im0, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors(c, True), 2)
#                                 cv2.putText(im0, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors(c, True), 2)

#                             # 打印结果
#                             class_counts = {}
#                             for cls_name, _ in results:
#                                 class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
#                             print_str = ", ".join([f"{v} {k}{'个' if k == '人' else '辆' if k == '公交车' else ''}{'s' if v > 1 and k not in ['人', '公交车'] else ''}"
#                                                    for k, v in class_counts.items()])
#                             LOGGER.info(f"检测结果: {print_str}")
#                         else:
#                             print_str = "未检测到目标"
#                             LOGGER.info(print_str)

#                     # 保存结果到文件
#                     output_file = output_dir / (img_path.stem + ".txt")
#                     with open(output_file, 'w') as f:
#                         f.write(print_str)
#                     LOGGER.info(f"检测结果已保存到: {output_file}")

#                     # 保存标注后的图片
#                     output_img_path = output_dir / img_path.name
#                     cv2.imwrite(str(output_img_path), im0)
#                     LOGGER.info(f"标注图片已保存到: {output_img_path}")

#             # 等待指定间隔后重新检查
#             time.sleep(opt.watch_interval)

#     except KeyboardInterrupt:
#         LOGGER.info("\n检测停止")


# def parse_opt():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="模型权重路径")
#     parser.add_argument("--source", type=str, default="data/images", help="监控的文件夹路径")
#     parser.add_argument("--output", type=str, default="output_results", help="保存检测结果和图片的文件夹路径")
#     parser.add_argument("--watch-interval", type=int, default=1,
#                         help="检查文件夹的时间间隔（秒），默认1秒")
#     parser.add_argument("--imgsz", nargs="+", type=int, default=[640], help="推理尺寸 (h,w)")
#     parser.add_argument("--conf-thres", type=float, default=0.25, help="置信度阈值")
#     parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU阈值")
#     parser.add_argument("--device", default="", help="设备 (cpu, 0, 1, ...)")
#     parser.add_argument("--half", action="store_true", help="使用FP16半精度推理")
#     parser.add_argument("--dnn", action="store_true", help="使用OpenCV DNN进行ONNX推理")
#     parser.add_argument("--data", type=str, default="data/coco128.yaml", help="数据集配置文件路径")
#     parser.add_argument("--augment", action="store_true", help="是否使用数据增强进行推理")
#     parser.add_argument("--classes", nargs="+", type=int, help="过滤指定类别的检测结果，例如 --classes 0 2 3 表示只检测类别0、2、3")
#     parser.add_argument("--agnostic_nms", action="store_true", help="是否使用类别无关的NMS")
#     parser.add_argument("--max_det", type=int, default=1000, help="每张图片的最大检测数量")
#     opt = parser.parse_args()
#     opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # 扩展尺寸格式
#     print_args(vars(opt))
#     return opt


# def print_args(args):
#     """打印参数配置"""
#     LOGGER.info("\n参数配置:")
#     for k, v in args.items():
#         LOGGER.info(f"  {k}: {v}")
#     LOGGER.info("\n")


# if __name__ == "__main__":
#     opt = parse_opt()
#     main(opt)





























##能输出识别结果的图片和txt文件
# Ultralytics YOLOv5 🚀, AGPL-3.0 license
import argparse
import os
import time
from pathlib import Path
import torch
import cv2
from models.common import DetectMultiBackend
from utils.general import (
    LOGGER,
    check_img_size,
    non_max_suppression,
    scale_boxes,
    xyxy2xywh,
)
from utils.torch_utils import select_device
from utils.dataloaders import LoadImages
from utils.plots import Annotator, colors


@torch.no_grad()
def main(opt):
    # 初始化模型
    device = select_device(opt.device)
    model = DetectMultiBackend(opt.weights, device=device, dnn=opt.dnn, data=opt.data, fp16=opt.half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(opt.imgsz, s=stride)  # 检查图像尺寸

    source_dir = Path(opt.source)
    output_dir = Path(opt.output)
    output_dir.mkdir(parents=True, exist_ok=True)  # 创建输出文件夹

    processed_files = set()  # 记录已处理的文件
    img_formats = [f".{f}" for f in ['bmp', 'dng', 'jpeg', 'jpg', 'mpo', 'png',
                                      'tif', 'tiff', 'webp', 'pfm']]  # 支持的图像格式

    LOGGER.info(f"开始监控文件夹: {source_dir}")
    LOGGER.info(f"等待新图片存入，按Ctrl+C停止...")

    try:
        while True:
            # 获取当前文件夹内所有符合格式的图片文件
            current_files = [f for f in source_dir.glob('*')
                             if f.suffix.lower() in img_formats and f.is_file()]

            # 过滤未处理的新文件（按修改时间排序，确保新文件先处理）
            new_files = [f for f in current_files if f.name not in processed_files]
            new_files.sort(key=lambda x: x.stat().st_mtime)  # 按修改时间排序

            for img_path in new_files:
                processed_files.add(img_path.name)  # 标记为已处理
                LOGGER.info(f"\n检测新图片: {img_path.name}")

                # 单图片处理流程
                dataset = LoadImages(str(img_path), img_size=imgsz, stride=stride, auto=pt)
                for path, im, im0s, _, _ in dataset:
                    im = torch.from_numpy(im).to(device)
                    im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
                    im /= 255  # 0 - 255 to 0.0 - 1.0
                    if len(im.shape) == 3:
                        im = im[None]  # 扩展batch维度

                    # 推理
                    pred = model(im, augment=opt.augment, visualize=False)

                    # NMS
                    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
                                               opt.classes, opt.agnostic_nms, opt.max_det)

                    # 处理检测结果
                    results = []
                    annotator = Annotator(im0s, line_width=2, example=str(names))
                    for det in pred:
                        if len(det):
                            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

                            # 收集检测结果
                            for *xyxy, conf, cls in det:
                                c = int(cls)
                                label = f'{names[c]} {conf:.2f}'
                                results.append((names[c], f"{conf:.2f}"))
                                # 修改为 box_label 方法
                                annotator.box_label(xyxy, label=label, color=colors(c, True))
                            im0 = annotator.result()

                            # 打印结果
                            class_counts = {}
                            for cls_name, _ in results:
                                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
                            print_str = ", ".join([f"{v} {k}{'s' if v > 1 else ''}"
                                                   for k, v in class_counts.items()])
                            LOGGER.info(f"检测结果: {print_str}")
                        else:
                            print_str = "未检测到目标"
                            im0 = im0s  # 若未检测到目标，使用原图
                            LOGGER.info(print_str)

                    # 保存结果到文件
                    output_file = output_dir / (img_path.stem + ".txt")
                    with open(output_file, 'w') as f:
                        f.write(print_str)
                    LOGGER.info(f"检测结果已保存到: {output_file}")

                    # 保存标注后的图片
                    output_img_path = output_dir / img_path.name
                    cv2.imwrite(str(output_img_path), im0)
                    LOGGER.info(f"标注图片已保存到: {output_img_path}")

            # 等待指定间隔后重新检查
            time.sleep(opt.watch_interval)

    except KeyboardInterrupt:
        LOGGER.info("\n检测停止")


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="模型权重路径")
    parser.add_argument("--source", type=str, default="data/images", help="监控的文件夹路径")
    parser.add_argument("--output", type=str, default="output_results", help="保存检测结果和图片的文件夹路径")
    parser.add_argument("--watch-interval", type=int, default=1,
                        help="检查文件夹的时间间隔（秒），默认1秒")
    parser.add_argument("--imgsz", nargs="+", type=int, default=[640], help="推理尺寸 (h,w)")
    parser.add_argument("--conf-thres", type=float, default=0.25, help="置信度阈值")
    parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU阈值")
    parser.add_argument("--device", default="", help="设备 (cpu, 0, 1, ...)")
    parser.add_argument("--half", action="store_true", help="使用FP16半精度推理")
    parser.add_argument("--dnn", action="store_true", help="使用OpenCV DNN进行ONNX推理")
    parser.add_argument("--data", type=str, default="data/coco128.yaml", help="数据集配置文件路径")
    parser.add_argument("--augment", action="store_true", help="是否使用数据增强进行推理")
    parser.add_argument("--classes", nargs="+", type=int, help="过滤指定类别的检测结果，例如 --classes 0 2 3 表示只检测类别0、2、3")
    parser.add_argument("--agnostic_nms", action="store_true", help="是否使用类别无关的NMS")
    parser.add_argument("--max_det", type=int, default=1000, help="每张图片的最大检测数量")
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # 扩展尺寸格式
    print_args(vars(opt))
    return opt


def print_args(args):
    """打印参数配置"""
    LOGGER.info("\n参数配置:")
    for k, v in args.items():
        LOGGER.info(f"  {k}: {v}")
    LOGGER.info("\n")


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)


























# 输出有带有结果的txt文件
# # Ultralytics YOLOv5 🚀, AGPL-3.0 license
# import argparse
# import os
# import time
# from pathlib import Path
# import torch
# from models.common import DetectMultiBackend
# from utils.general import (
#     LOGGER,
#     check_img_size,
#     non_max_suppression,
#     scale_boxes,
#     xyxy2xywh,
# )
# from utils.torch_utils import select_device
# from utils.dataloaders import LoadImages
# from utils.plots import Annotator, colors


# @torch.no_grad()
# def main(opt):
#     # 初始化模型
#     device = select_device(opt.device)
#     model = DetectMultiBackend(opt.weights, device=device, dnn=opt.dnn, data=opt.data, fp16=opt.half)
#     stride, names, pt = model.stride, model.names, model.pt
#     imgsz = check_img_size(opt.imgsz, s=stride)  # 检查图像尺寸

#     source_dir = Path(opt.source)
#     output_dir = Path(opt.output)
#     output_dir.mkdir(parents=True, exist_ok=True)  # 创建输出文件夹

#     processed_files = set()  # 记录已处理的文件
#     img_formats = [f".{f}" for f in ['bmp', 'dng', 'jpeg', 'jpg', 'mpo', 'png',
#                                       'tif', 'tiff', 'webp', 'pfm']]  # 支持的图像格式

#     LOGGER.info(f"开始监控文件夹: {source_dir}")
#     LOGGER.info(f"等待新图片存入，按Ctrl+C停止...")

#     try:
#         while True:
#             # 获取当前文件夹内所有符合格式的图片文件
#             current_files = [f for f in source_dir.glob('*')
#                              if f.suffix.lower() in img_formats and f.is_file()]

#             # 过滤未处理的新文件（按修改时间排序，确保新文件先处理）
#             new_files = [f for f in current_files if f.name not in processed_files]
#             new_files.sort(key=lambda x: x.stat().st_mtime)  # 按修改时间排序

#             for img_path in new_files:
#                 processed_files.add(img_path.name)  # 标记为已处理
#                 LOGGER.info(f"\n检测新图片: {img_path.name}")

#                 # 单图片处理流程
#                 dataset = LoadImages(str(img_path), img_size=imgsz, stride=stride, auto=pt)
#                 for path, im, im0s, _, _ in dataset:
#                     im = torch.from_numpy(im).to(device)
#                     im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
#                     im /= 255  # 0 - 255 to 0.0 - 1.0
#                     if len(im.shape) == 3:
#                         im = im[None]  # 扩展batch维度

#                     # 推理
#                     pred = model(im, augment=opt.augment, visualize=False)

#                     # NMS
#                     pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
#                                                opt.classes, opt.agnostic_nms, opt.max_det)

#                     # 处理检测结果
#                     results = []
#                     for det in pred:
#                         if len(det):
#                             det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

#                             # 收集检测结果
#                             for *xyxy, conf, cls in det:
#                                 c = int(cls)
#                                 results.append((names[c], f"{conf:.2f}"))

#                             # 打印结果
#                             class_counts = {}
#                             for cls_name, _ in results:
#                                 class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
#                             print_str = ", ".join([f"{v} {k}{'s' if v > 1 else ''}"
#                                                    for k, v in class_counts.items()])
#                             LOGGER.info(f"检测结果: {print_str}")
#                         else:
#                             print_str = "未检测到目标"
#                             LOGGER.info(print_str)

#                     # 保存结果到文件
#                     output_file = output_dir / (img_path.stem + ".txt")
#                     with open(output_file, 'w') as f:
#                         f.write(print_str)
#                     LOGGER.info(f"检测结果已保存到: {output_file}")

#             # 等待指定间隔后重新检查
#             time.sleep(opt.watch_interval)

#     except KeyboardInterrupt:
#         LOGGER.info("\n检测停止")


# def parse_opt():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="模型权重路径")
#     parser.add_argument("--source", type=str, default="data/images", help="监控的文件夹路径")
#     parser.add_argument("--output", type=str, default="output_results", help="保存检测结果的文件夹路径")
#     parser.add_argument("--watch-interval", type=int, default=1,
#                         help="检查文件夹的时间间隔（秒），默认1秒")
#     parser.add_argument("--imgsz", nargs="+", type=int, default=[640], help="推理尺寸 (h,w)")
#     parser.add_argument("--conf-thres", type=float, default=0.25, help="置信度阈值")
#     parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU阈值")
#     parser.add_argument("--device", default="", help="设备 (cpu, 0, 1, ...)")
#     parser.add_argument("--half", action="store_true", help="使用FP16半精度推理")
#     parser.add_argument("--dnn", action="store_true", help="使用OpenCV DNN进行ONNX推理")
#     parser.add_argument("--data", type=str, default="data/coco128.yaml", help="数据集配置文件路径")
#     parser.add_argument("--augment", action="store_true", help="是否使用数据增强进行推理")
#     parser.add_argument("--classes", nargs="+", type=int, help="过滤指定类别的检测结果，例如 --classes 0 2 3 表示只检测类别0、2、3")
#     parser.add_argument("--agnostic_nms", action="store_true", help="是否使用类别无关的NMS")
#     parser.add_argument("--max_det", type=int, default=1000, help="每张图片的最大检测数量")
#     opt = parser.parse_args()
#     opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # 扩展尺寸格式
#     print_args(vars(opt))
#     return opt


# def print_args(args):
#     """打印参数配置"""
#     LOGGER.info("\n参数配置:")
#     for k, v in args.items():
#         LOGGER.info(f"  {k}: {v}")
#     LOGGER.info("\n")


# if __name__ == "__main__":
#     opt = parse_opt()
#     main(opt)

























# 有检测没有输出保存文件夹
# # Ultralytics YOLOv5 🚀, AGPL-3.0 license
# import argparse
# import os
# import time
# from pathlib import Path
# import torch
# from models.common import DetectMultiBackend
# from utils.general import (
#     LOGGER,
#     check_img_size,
#     non_max_suppression,
#     scale_boxes,
#     xyxy2xywh,
# )
# from utils.torch_utils import select_device
# from utils.dataloaders import LoadImages
# from utils.plots import Annotator, colors


# @torch.no_grad()
# def main(opt):
#     # 初始化模型
#     device = select_device(opt.device)
#     model = DetectMultiBackend(opt.weights, device=device, dnn=opt.dnn, data=opt.data, fp16=opt.half)
#     stride, names, pt = model.stride, model.names, model.pt
#     imgsz = check_img_size(opt.imgsz, s=stride)  # 检查图像尺寸

#     source_dir = Path(opt.source)
#     processed_files = set()  # 记录已处理的文件
#     img_formats = [f".{f}" for f in ['bmp', 'dng', 'jpeg', 'jpg', 'mpo', 'png',
#                                       'tif', 'tiff', 'webp', 'pfm']]  # 支持的图像格式

#     LOGGER.info(f"开始监控文件夹: {source_dir}")
#     LOGGER.info(f"等待新图片存入，按Ctrl+C停止...")

#     try:
#         while True:
#             # 获取当前文件夹内所有符合格式的图片文件
#             current_files = [f for f in source_dir.glob('*')
#                              if f.suffix.lower() in img_formats and f.is_file()]

#             # 过滤未处理的新文件（按修改时间排序，确保新文件先处理）
#             new_files = [f for f in current_files if f.name not in processed_files]
#             new_files.sort(key=lambda x: x.stat().st_mtime)  # 按修改时间排序

#             for img_path in new_files:
#                 processed_files.add(img_path.name)  # 标记为已处理
#                 LOGGER.info(f"\n检测新图片: {img_path.name}")

#                 # 单图片处理流程
#                 dataset = LoadImages(str(img_path), img_size=imgsz, stride=stride, auto=pt)
#                 for path, im, im0s, _, _ in dataset:
#                     im = torch.from_numpy(im).to(device)
#                     im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
#                     im /= 255  # 0 - 255 to 0.0 - 1.0
#                     if len(im.shape) == 3:
#                         im = im[None]  # 扩展batch维度

#                     # 推理
#                     pred = model(im, augment=opt.augment, visualize=False)

#                     # NMS
#                     pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
#                                                opt.classes, opt.agnostic_nms, opt.max_det)

#                     # 处理检测结果
#                     for det in pred:
#                         if len(det):
#                             det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

#                             # 收集检测结果
#                             results = []
#                             for *xyxy, conf, cls in det:
#                                 c = int(cls)
#                                 results.append((names[c], f"{conf:.2f}"))

#                             # 打印结果
#                             class_counts = {}
#                             for cls_name, _ in results:
#                                 class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
#                             print_str = ", ".join([f"{v} {k}{'s' if v > 1 else ''}"
#                                                    for k, v in class_counts.items()])
#                             LOGGER.info(f"检测结果: {print_str}")

#                         else:
#                             LOGGER.info("未检测到目标")

#             # 等待指定间隔后重新检查
#             time.sleep(opt.watch_interval)

#     except KeyboardInterrupt:
#         LOGGER.info("\n检测停止")


# def parse_opt():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="模型权重路径")
#     parser.add_argument("--source", type=str, default="data/images", help="监控的文件夹路径")
#     parser.add_argument("--watch-interval", type=int, default=1,
#                         help="检查文件夹的时间间隔（秒），默认1秒")
#     parser.add_argument("--imgsz", nargs="+", type=int, default=[640], help="推理尺寸 (h,w)")
#     parser.add_argument("--conf-thres", type=float, default=0.25, help="置信度阈值")
#     parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU阈值")
#     parser.add_argument("--device", default="", help="设备 (cpu, 0, 1, ...)")
#     parser.add_argument("--half", action="store_true", help="使用FP16半精度推理")
#     parser.add_argument("--dnn", action="store_true", help="使用OpenCV DNN进行ONNX推理")
#     parser.add_argument("--data", type=str, default="data/coco128.yaml", help="数据集配置文件路径")
#     parser.add_argument("--augment", action="store_true", help="是否使用数据增强进行推理")
#     parser.add_argument("--classes", nargs="+", type=int, help="过滤指定类别的检测结果，例如 --classes 0 2 3 表示只检测类别0、2、3")
#     parser.add_argument("--agnostic_nms", action="store_true", help="是否使用类别无关的NMS")
#     parser.add_argument("--max_det", type=int, default=1000, help="每张图片的最大检测数量")
#     opt = parser.parse_args()
#     opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # 扩展尺寸格式
#     print_args(vars(opt))
#     return opt


# def print_args(args):
#     """打印参数配置"""
#     LOGGER.info("\n参数配置:")
#     for k, v in args.items():
#         LOGGER.info(f"  {k}: {v}")
#     LOGGER.info("\n")


# if __name__ == "__main__":
#     opt = parse_opt()
#     main(opt)


















# # Ultralytics YOLOv5 🚀, AGPL-3.0 license
# """
# Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

# Usage - sources:
#     $ python detect.py --weights yolov5s.pt --source 0                               # webcam
#                                                      img.jpg                         # image
#                                                      vid.mp4                         # video
#                                                      screen                          # screenshot
#                                                      path/                           # directory
#                                                      list.txt                        # list of images
#                                                      list.streams                    # list of streams
#                                                      'path/*.jpg'                    # glob
#                                                      'https://youtu.be/LNwODJXcvt4'  # YouTube
#                                                      'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

# Usage - formats:
#     $ python detect.py --weights yolov5s.pt                 # PyTorch
#                                  yolov5s.torchscript        # TorchScript
#                                  yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
#                                  yolov5s_openvino_model     # OpenVINO
#                                  yolov5s.engine             # TensorRT
#                                  yolov5s.mlpackage          # CoreML (macOS-only)
#                                  yolov5s_saved_model        # TensorFlow SavedModel
#                                  yolov5s.pb                 # TensorFlow GraphDef
#                                  yolov5s.tflite             # TensorFlow Lite
#                                  yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
#                                  yolov5s_paddle_model       # PaddlePaddle
# """

# import argparse
# import csv
# import os
# import platform
# import sys
# from pathlib import Path

# import torch

# FILE = Path(__file__).resolve()
# ROOT = FILE.parents[0]  # YOLOv5 root directory
# if str(ROOT) not in sys.path:
#     sys.path.append(str(ROOT))  # add ROOT to PATH
# ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# from ultralytics.utils.plotting import Annotator, colors, save_one_box

# from models.common import DetectMultiBackend
# from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
# from utils.general import (
#     LOGGER,
#     Profile,
#     check_file,
#     check_img_size,
#     check_imshow,
#     check_requirements,
#     colorstr,
#     cv2,
#     increment_path,
#     non_max_suppression,
#     print_args,
#     scale_boxes,
#     strip_optimizer,
#     xyxy2xywh,
# )
# from utils.torch_utils import select_device, smart_inference_mode


# @smart_inference_mode()
# def run(
#     weights=ROOT / "yolov5s.pt",  # model path or triton URL
#     source=ROOT / "data/images",  # file/dir/URL/glob/screen/0(webcam)
#     data=ROOT / "data/coco128.yaml",  # dataset.yaml path
#     imgsz=(640, 640),  # inference size (height, width)
#     conf_thres=0.25,  # confidence threshold
#     iou_thres=0.45,  # NMS IOU threshold
#     max_det=1000,  # maximum detections per image
#     device="",  # cuda device, i.e. 0 or 0,1,2,3 or cpu
#     view_img=False,  # show results
#     save_txt=False,  # save results to *.txt
#     save_format=0,  # save boxes coordinates in YOLO format or Pascal-VOC format (0 for YOLO and 1 for Pascal-VOC)
#     save_csv=False,  # save results in CSV format
#     save_conf=False,  # save confidences in --save-txt labels
#     save_crop=False,  # save cropped prediction boxes
#     nosave=False,  # do not save images/videos
#     classes=None,  # filter by class: --class 0, or --class 0 2 3
#     agnostic_nms=False,  # class-agnostic NMS
#     augment=False,  # augmented inference
#     visualize=False,  # visualize features
#     update=False,  # update all models
#     project=ROOT / "runs/detect",  # save results to project/name
#     name="exp",  # save results to project/name
#     exist_ok=False,  # existing project/name ok, do not increment
#     line_thickness=3,  # bounding box thickness (pixels)
#     hide_labels=False,  # hide labels
#     hide_conf=False,  # hide confidences
#     half=False,  # use FP16 half-precision inference
#     dnn=False,  # use OpenCV DNN for ONNX inference
#     vid_stride=1,  # video frame-rate stride
# ):
#     """
#     Runs YOLOv5 detection inference on various sources like images, videos, directories, streams, etc.

#     Args:
#         weights (str | Path): Path to the model weights file or a Triton URL. Default is 'yolov5s.pt'.
#         source (str | Path): Input source, which can be a file, directory, URL, glob pattern, screen capture, or webcam
#             index. Default is 'data/images'.
#         data (str | Path): Path to the dataset YAML file. Default is 'data/coco128.yaml'.
#         imgsz (tuple[int, int]): Inference image size as a tuple (height, width). Default is (640, 640).
#         conf_thres (float): Confidence threshold for detections. Default is 0.25.
#         iou_thres (float): Intersection Over Union (IOU) threshold for non-max suppression. Default is 0.45.
#         max_det (int): Maximum number of detections per image. Default is 1000.
#         device (str): CUDA device identifier (e.g., '0' or '0,1,2,3') or 'cpu'. Default is an empty string, which uses the
#             best available device.
#         view_img (bool): If True, display inference results using OpenCV. Default is False.
#         save_txt (bool): If True, save results in a text file. Default is False.
#         save_csv (bool): If True, save results in a CSV file. Default is False.
#         save_conf (bool): If True, include confidence scores in the saved results. Default is False.
#         save_crop (bool): If True, save cropped prediction boxes. Default is False.
#         nosave (bool): If True, do not save inference images or videos. Default is False.
#         classes (list[int]): List of class indices to filter detections by. Default is None.
#         agnostic_nms (bool): If True, perform class-agnostic non-max suppression. Default is False.
#         augment (bool): If True, use augmented inference. Default is False.
#         visualize (bool): If True, visualize feature maps. Default is False.
#         update (bool): If True, update all models' weights. Default is False.
#         project (str | Path): Directory to save results. Default is 'runs/detect'.
#         name (str): Name of the current experiment; used to create a subdirectory within 'project'. Default is 'exp'.
#         exist_ok (bool): If True, existing directories with the same name are reused instead of being incremented. Default is
#             False.
#         line_thickness (int): Thickness of bounding box lines in pixels. Default is 3.
#         hide_labels (bool): If True, do not display labels on bounding boxes. Default is False.
#         hide_conf (bool): If True, do not display confidence scores on bounding boxes. Default is False.
#         half (bool): If True, use FP16 half-precision inference. Default is False.
#         dnn (bool): If True, use OpenCV DNN backend for ONNX inference. Default is False.
#         vid_stride (int): Stride for processing video frames, to skip frames between processing. Default is 1.

#     Returns:
#         None

#     Examples:
#         ```python
#         from ultralytics import run

#         # Run inference on an image
#         run(source='data/images/example.jpg', weights='yolov5s.pt', device='0')

#         # Run inference on a video with specific confidence threshold
#         run(source='data/videos/example.mp4', weights='yolov5s.pt', conf_thres=0.4, device='0')
#         ```
#     """
#     source = str(source)
#     save_img = not nosave and not source.endswith(".txt")  # save inference images
#     is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
#     is_url = source.lower().startswith(("rtsp://", "rtmp://", "http://", "https://"))
#     webcam = source.isnumeric() or source.endswith(".streams") or (is_url and not is_file)
#     screenshot = source.lower().startswith("screen")
#     if is_url and is_file:
#         source = check_file(source)  # download

#     # Directories
#     save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
#     (save_dir / "labels" if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

#     # Load model
#     device = select_device(device)
#     model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
#     stride, names, pt = model.stride, model.names, model.pt
#     imgsz = check_img_size(imgsz, s=stride)  # check image size

#     # Dataloader
#     bs = 1  # batch_size
#     if webcam:
#         view_img = check_imshow(warn=True)
#         dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
#         bs = len(dataset)
#     elif screenshot:
#         dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
#     else:
#         dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
#     vid_path, vid_writer = [None] * bs, [None] * bs

#     # Run inference
#     model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
#     seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
#     for path, im, im0s, vid_cap, s in dataset:
#         with dt[0]:
#             im = torch.from_numpy(im).to(model.device)
#             im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
#             im /= 255  # 0 - 255 to 0.0 - 1.0
#             if len(im.shape) == 3:
#                 im = im[None]  # expand for batch dim
#             if model.xml and im.shape[0] > 1:
#                 ims = torch.chunk(im, im.shape[0], 0)

#         # Inference
#         with dt[1]:
#             visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
#             if model.xml and im.shape[0] > 1:
#                 pred = None
#                 for image in ims:
#                     if pred is None:
#                         pred = model(image, augment=augment, visualize=visualize).unsqueeze(0)
#                     else:
#                         pred = torch.cat((pred, model(image, augment=augment, visualize=visualize).unsqueeze(0)), dim=0)
#                 pred = [pred, None]
#             else:
#                 pred = model(im, augment=augment, visualize=visualize)
#         # NMS
#         with dt[2]:
#             pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

#         # Second-stage classifier (optional)
#         # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

#         # Define the path for the CSV file
#         csv_path = save_dir / "predictions.csv"

#         # Create or append to the CSV file
#         def write_to_csv(image_name, prediction, confidence):
#             """Writes prediction data for an image to a CSV file, appending if the file exists."""
#             data = {"Image Name": image_name, "Prediction": prediction, "Confidence": confidence}
#             with open(csv_path, mode="a", newline="") as f:
#                 writer = csv.DictWriter(f, fieldnames=data.keys())
#                 if not csv_path.is_file():
#                     writer.writeheader()
#                 writer.writerow(data)

#         # Process predictions
#         for i, det in enumerate(pred):  # per image
#             seen += 1
#             if webcam:  # batch_size >= 1
#                 p, im0, frame = path[i], im0s[i].copy(), dataset.count
#                 s += f"{i}: "
#             else:
#                 p, im0, frame = path, im0s.copy(), getattr(dataset, "frame", 0)

#             p = Path(p)  # to Path
#             save_path = str(save_dir / p.name)  # im.jpg
#             txt_path = str(save_dir / "labels" / p.stem) + ("" if dataset.mode == "image" else f"_{frame}")  # im.txt
#             s += "{:g}x{:g} ".format(*im.shape[2:])  # print string
#             gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
#             imc = im0.copy() if save_crop else im0  # for save_crop
#             annotator = Annotator(im0, line_width=line_thickness, example=str(names))
#             if len(det):
#                 # Rescale boxes from img_size to im0 size
#                 det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

#                 # Print results
#                 for c in det[:, 5].unique():
#                     n = (det[:, 5] == c).sum()  # detections per class
#                     s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

#                 # Write results
#                 for *xyxy, conf, cls in reversed(det):
#                     c = int(cls)  # integer class
#                     label = names[c] if hide_conf else f"{names[c]}"
#                     confidence = float(conf)
#                     confidence_str = f"{confidence:.2f}"

#                     if save_csv:
#                         write_to_csv(p.name, label, confidence_str)

#                     if save_txt:  # Write to file
#                         if save_format == 0:
#                             coords = (
#                                 (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
#                             )  # normalized xywh
#                         else:
#                             coords = (torch.tensor(xyxy).view(1, 4) / gn).view(-1).tolist()  # xyxy
#                         line = (cls, *coords, conf) if save_conf else (cls, *coords)  # label format
#                         with open(f"{txt_path}.txt", "a") as f:
#                             f.write(("%g " * len(line)).rstrip() % line + "\n")

#                     if save_img or save_crop or view_img:  # Add bbox to image
#                         c = int(cls)  # integer class
#                         label = None if hide_labels else (names[c] if hide_conf else f"{names[c]} {conf:.2f}")
#                         annotator.box_label(xyxy, label, color=colors(c, True))
#                     if save_crop:
#                         save_one_box(xyxy, imc, file=save_dir / "crops" / names[c] / f"{p.stem}.jpg", BGR=True)

#             # Stream results
#             im0 = annotator.result()
#             if view_img:
#                 if platform.system() == "Linux" and p not in windows:
#                     windows.append(p)
#                     cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
#                     cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
#                 cv2.imshow(str(p), im0)
#                 cv2.waitKey(1)  # 1 millisecond

#             # Save results (image with detections)
#             if save_img:
#                 if dataset.mode == "image":
#                     cv2.imwrite(save_path, im0)
#                 else:  # 'video' or 'stream'
#                     if vid_path[i] != save_path:  # new video
#                         vid_path[i] = save_path
#                         if isinstance(vid_writer[i], cv2.VideoWriter):
#                             vid_writer[i].release()  # release previous video writer
#                         if vid_cap:  # video
#                             fps = vid_cap.get(cv2.CAP_PROP_FPS)
#                             w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#                             h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#                         else:  # stream
#                             fps, w, h = 30, im0.shape[1], im0.shape[0]
#                         save_path = str(Path(save_path).with_suffix(".mp4"))  # force *.mp4 suffix on results videos
#                         vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
#                     vid_writer[i].write(im0)

#         # Print time (inference-only)
#         LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

#     # Print results
#     t = tuple(x.t / seen * 1e3 for x in dt)  # speeds per image
#     LOGGER.info(f"Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}" % t)
#     if save_txt or save_img:
#         s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ""
#         LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
#     if update:
#         strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)


# def parse_opt():
#     """
#     Parse command-line arguments for YOLOv5 detection, allowing custom inference options and model configurations.

#     Args:
#         --weights (str | list[str], optional): Model path or Triton URL. Defaults to ROOT / 'yolov5s.pt'.
#         --source (str, optional): File/dir/URL/glob/screen/0(webcam). Defaults to ROOT / 'data/images'.
#         --data (str, optional): Dataset YAML path. Provides dataset configuration information.
#         --imgsz (list[int], optional): Inference size (height, width). Defaults to [640].
#         --conf-thres (float, optional): Confidence threshold. Defaults to 0.25.
#         --iou-thres (float, optional): NMS IoU threshold. Defaults to 0.45.
#         --max-det (int, optional): Maximum number of detections per image. Defaults to 1000.
#         --device (str, optional): CUDA device, i.e., '0' or '0,1,2,3' or 'cpu'. Defaults to "".
#         --view-img (bool, optional): Flag to display results. Defaults to False.
#         --save-txt (bool, optional): Flag to save results to *.txt files. Defaults to False.
#         --save-csv (bool, optional): Flag to save results in CSV format. Defaults to False.
#         --save-conf (bool, optional): Flag to save confidences in labels saved via --save-txt. Defaults to False.
#         --save-crop (bool, optional): Flag to save cropped prediction boxes. Defaults to False.
#         --nosave (bool, optional): Flag to prevent saving images/videos. Defaults to False.
#         --classes (list[int], optional): List of classes to filter results by, e.g., '--classes 0 2 3'. Defaults to None.
#         --agnostic-nms (bool, optional): Flag for class-agnostic NMS. Defaults to False.
#         --augment (bool, optional): Flag for augmented inference. Defaults to False.
#         --visualize (bool, optional): Flag for visualizing features. Defaults to False.
#         --update (bool, optional): Flag to update all models in the model directory. Defaults to False.
#         --project (str, optional): Directory to save results. Defaults to ROOT / 'runs/detect'.
#         --name (str, optional): Sub-directory name for saving results within --project. Defaults to 'exp'.
#         --exist-ok (bool, optional): Flag to allow overwriting if the project/name already exists. Defaults to False.
#         --line-thickness (int, optional): Thickness (in pixels) of bounding boxes. Defaults to 3.
#         --hide-labels (bool, optional): Flag to hide labels in the output. Defaults to False.
#         --hide-conf (bool, optional): Flag to hide confidences in the output. Defaults to False.
#         --half (bool, optional): Flag to use FP16 half-precision inference. Defaults to False.
#         --dnn (bool, optional): Flag to use OpenCV DNN for ONNX inference. Defaults to False.
#         --vid-stride (int, optional): Video frame-rate stride, determining the number of frames to skip in between
#             consecutive frames. Defaults to 1.

#     Returns:
#         argparse.Namespace: Parsed command-line arguments as an argparse.Namespace object.

#     Example:
#         ```python
#         from ultralytics import YOLOv5
#         args = YOLOv5.parse_opt()
#         ```
#     """
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--weights", nargs="+", type=str, default=ROOT / "yolov5s.pt", help="model path or triton URL")
#     parser.add_argument("--source", type=str, default=ROOT / "data/images", help="file/dir/URL/glob/screen/0(webcam)")
#     parser.add_argument("--data", type=str, default=ROOT / "data/coco128.yaml", help="(optional) dataset.yaml path")
#     parser.add_argument("--imgsz", "--img", "--img-size", nargs="+", type=int, default=[640], help="inference size h,w")
#     parser.add_argument("--conf-thres", type=float, default=0.25, help="confidence threshold")
#     parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU threshold")
#     parser.add_argument("--max-det", type=int, default=1000, help="maximum detections per image")
#     parser.add_argument("--device", default="", help="cuda device, i.e. 0 or 0,1,2,3 or cpu")
#     parser.add_argument("--view-img", action="store_true", help="show results")
#     parser.add_argument("--save-txt", action="store_true", help="save results to *.txt")
#     parser.add_argument(
#         "--save-format",
#         type=int,
#         default=0,
#         help="whether to save boxes coordinates in YOLO format or Pascal-VOC format when save-txt is True, 0 for YOLO and 1 for Pascal-VOC",
#     )
#     parser.add_argument("--save-csv", action="store_true", help="save results in CSV format")
#     parser.add_argument("--save-conf", action="store_true", help="save confidences in --save-txt labels")
#     parser.add_argument("--save-crop", action="store_true", help="save cropped prediction boxes")
#     parser.add_argument("--nosave", action="store_true", help="do not save images/videos")
#     parser.add_argument("--classes", nargs="+", type=int, help="filter by class: --classes 0, or --classes 0 2 3")
#     parser.add_argument("--agnostic-nms", action="store_true", help="class-agnostic NMS")
#     parser.add_argument("--augment", action="store_true", help="augmented inference")
#     parser.add_argument("--visualize", action="store_true", help="visualize features")
#     parser.add_argument("--update", action="store_true", help="update all models")
#     parser.add_argument("--project", default=ROOT / "runs/detect", help="save results to project/name")
#     parser.add_argument("--name", default="exp", help="save results to project/name")
#     parser.add_argument("--exist-ok", action="store_true", help="existing project/name ok, do not increment")
#     parser.add_argument("--line-thickness", default=3, type=int, help="bounding box thickness (pixels)")
#     parser.add_argument("--hide-labels", default=False, action="store_true", help="hide labels")
#     parser.add_argument("--hide-conf", default=False, action="store_true", help="hide confidences")
#     parser.add_argument("--half", action="store_true", help="use FP16 half-precision inference")
#     parser.add_argument("--dnn", action="store_true", help="use OpenCV DNN for ONNX inference")
#     parser.add_argument("--vid-stride", type=int, default=1, help="video frame-rate stride")
#     opt = parser.parse_args()
#     opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
#     print_args(vars(opt))
#     return opt


# def main(opt):
#     """
#     Executes YOLOv5 model inference based on provided command-line arguments, validating dependencies before running.

#     Args:
#         opt (argparse.Namespace): Command-line arguments for YOLOv5 detection. See function `parse_opt` for details.

#     Returns:
#         None

#     Note:
#         This function performs essential pre-execution checks and initiates the YOLOv5 detection process based on user-specified
#         options. Refer to the usage guide and examples for more information about different sources and formats at:
#         https://github.com/ultralytics/ultralytics

#     Example usage:

#     ```python
#     if __name__ == "__main__":
#         opt = parse_opt()
#         main(opt)
#     ```
#     """
#     check_requirements(ROOT / "requirements.txt", exclude=("tensorboard", "thop"))
#     run(**vars(opt))


# if __name__ == "__main__":
#     opt = parse_opt()
#     main(opt)
