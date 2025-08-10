## 使用方法：

##     文件夹监控识别：python3 auto_detect.py 

## 实时视频识别：使用默认摄像头：python3 auto_detect.py --mode video --source 0
## 使用视频文件：# python scriauto_detectpt.py --mode video --source path/to/video.mp4

## Ultralytics YOLOv5 🚀, AGPL - 3.0 license
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
from utils.plots import colors

# 假设这是中文类别名称映射，你可以根据实际情况修改
CHINESE_CLASS_NAMES = {
    'person': '人',
    'bus': '公交车',
    # 可以继续添加其他类别映射
}


def init_model(opt):
    # 初始化模型
    device = select_device(opt.device)
    model = DetectMultiBackend(opt.weights, device=device, dnn=opt.dnn, data=opt.data, fp16=opt.half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(opt.imgsz, s=stride)  # 检查图像尺寸
    return model, device, stride, names, pt, imgsz


@torch.no_grad()
def detect_folder(opt, model, device, stride, names, pt, imgsz):
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
                try:
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

                        # 记录推理开始时间
                        start_time = time.time()

                        # 推理
                        pred = model(im, augment=opt.augment, visualize=False)

                        # 记录推理结束时间
                        end_time = time.time()
                        inference_time = end_time - start_time
                        LOGGER.info(f"识别用时: {inference_time:.4f} 秒")

                        # NMS
                        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
                                                   opt.classes, opt.agnostic_nms, opt.max_det)

                        # 处理检测结果
                        results = []
                        im0 = im0s.copy()
                        for det in pred:
                            if len(det):
                                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

                                # 收集检测结果
                                for *xyxy, conf, cls in det:
                                    c = int(cls)
                                    original_name = names[c]
                                    chinese_name = CHINESE_CLASS_NAMES.get(original_name, original_name)
                                    label = f'{chinese_name} {conf:.2f}'
                                    results.append((chinese_name, f"{conf:.2f}"))

                                    # 手动绘制边界框和标签
                                    xyxy = [int(i) for i in xyxy]
                                    cv2.rectangle(im0, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors(c, True), 2)
                                    cv2.putText(im0, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                                colors(c, True), 2)

                                # 打印结果
                                class_counts = {}
                                for cls_name, _ in results:
                                    class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

                                # 修正中文输出
                                print_str_list = []
                                for k, v in class_counts.items():
                                    if k == '人':
                                        print_str_list.append(f"{v} {k}")
                                    elif k == '公交车':
                                        print_str_list.append(f"{v} {k}")
                                    else:
                                        print_str_list.append(f"{v} {k}{'s' if v > 1 and k not in ['人', '公交车'] else ''}")
                                print_str = ", ".join(print_str_list)

                                LOGGER.info(f"检测结果: {print_str}")
                            else:
                                print_str = "未检测到目标"
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
                except AssertionError as e:
                    LOGGER.error(f"处理图片 {img_path.name} 时出错: {e}")
                except Exception as e:
                    LOGGER.error(f"处理图片 {img_path.name} 时发生未知错误: {e}")

            # 等待指定间隔后重新检查
            time.sleep(opt.watch_interval)

    except KeyboardInterrupt:
        LOGGER.info("\n检测停止")


@torch.no_grad()
def detect_video(opt, model, device, stride, names, pt, imgsz):
    # 打开视频流
    if opt.source.isdigit():
        cap = cv2.VideoCapture(int(opt.source))  # 摄像头设备
    else:
        cap = cv2.VideoCapture(opt.source)  # 视频文件

    if not cap.isOpened():
        LOGGER.error("无法打开视频流")
        return

    LOGGER.info("开始实时目标检测，按 'q' 键退出...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                LOGGER.info("视频流结束")
                break

            # 单帧处理流程
            im = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = cv2.resize(im, (imgsz[0], imgsz[1]))
            im = torch.from_numpy(im).to(device)
            im = im.permute(2, 0, 1).unsqueeze(0).float() / 255.0  # uint8 to fp32

            # 记录推理开始时间
            start_time = time.time()

            # 推理
            pred = model(im, augment=opt.augment, visualize=False)

            # 记录推理结束时间
            end_time = time.time()
            inference_time = end_time - start_time
            LOGGER.info(f"识别用时: {inference_time:.4f} 秒")

            # NMS
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
                                       opt.classes, opt.agnostic_nms, opt.max_det)

            # 处理检测结果
            results = []
            im0 = frame.copy()
            for det in pred:
                if len(det):
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                    # 收集检测结果
                    for *xyxy, conf, cls in det:
                        c = int(cls)
                        original_name = names[c]
                        chinese_name = CHINESE_CLASS_NAMES.get(original_name, original_name)
                        label = f'{chinese_name} {conf:.2f}'
                        results.append((chinese_name, f"{conf:.2f}"))

                        # 手动绘制边界框和标签
                        xyxy = [int(i) for i in xyxy]
                        cv2.rectangle(im0, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors(c, True), 2)
                        cv2.putText(im0, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    colors(c, True), 2)

                    # 打印结果
                    class_counts = {}
                    for cls_name, _ in results:
                        class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

                    # 修正中文输出
                    print_str_list = []
                    for k, v in class_counts.items():
                        if k == '人':
                            print_str_list.append(f"{v} {k}")
                        elif k == '公交车':
                            print_str_list.append(f"{v} {k}")
                        else:
                            print_str_list.append(f"{v} {k}{'s' if v > 1 and k not in ['人', '公交车'] else ''}")
                    print_str = ", ".join(print_str_list)

                    LOGGER.info(f"检测结果: {print_str}")
                else:
                    print_str = "未检测到目标"
                    LOGGER.info(print_str)

            # 显示结果
            cv2.imshow('YOLOv5 Real-time Detection', im0)

            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        LOGGER.info("\n检测停止")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="模型权重路径")
    parser.add_argument("--source", type=str, default="data/images", help="监控的文件夹路径或视频流源，0 表示默认摄像头，也可以是视频文件路径")
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
    parser.add_argument("--mode", type=str, default="folder", choices=["folder", "video"],
                        help="选择识别模式，'folder' 表示监控文件夹，'video' 表示实时视频识别")
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # 扩展尺寸格式
    return opt


if __name__ == "__main__":
    opt = parse_opt()
    model, device, stride, names, pt, imgsz = init_model(opt)

    if opt.mode == "folder":
        detect_folder(opt, model, device, stride, names, pt, imgsz)
    elif opt.mode == "video":
        detect_video(opt, model, device, stride, names, pt, imgsz)





























## 可以不需要roscore
# # # Ultralytics YOLOv5 🚀, AGPL - 3.0 license
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
#                 try:
#                     processed_files.add(img_path.name)  # 标记为已处理
#                     LOGGER.info(f"\n检测新图片: {img_path.name}")

#                     # 单图片处理流程
#                     dataset = LoadImages(str(img_path), img_size=imgsz, stride=stride, auto=pt)
#                     for path, im, im0s, _, _ in dataset:
#                         im = torch.from_numpy(im).to(device)
#                         im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
#                         im /= 255  # 0 - 255 to 0.0 - 1.0
#                         if len(im.shape) == 3:
#                             im = im[None]  # 扩展batch维度

#                         # 记录推理开始时间
#                         start_time = time.time()

#                         # 推理
#                         pred = model(im, augment=opt.augment, visualize=False)

#                         # 记录推理结束时间
#                         end_time = time.time()
#                         inference_time = end_time - start_time
#                         LOGGER.info(f"识别用时: {inference_time:.4f} 秒")

#                         # NMS
#                         pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
#                                                    opt.classes, opt.agnostic_nms, opt.max_det)

#                         # 处理检测结果
#                         results = []
#                         im0 = im0s.copy()
#                         for det in pred:
#                             if len(det):
#                                 det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

#                                 # 收集检测结果
#                                 for *xyxy, conf, cls in det:
#                                     c = int(cls)
#                                     original_name = names[c]
#                                     chinese_name = CHINESE_CLASS_NAMES.get(original_name, original_name)
#                                     label = f'{chinese_name} {conf:.2f}'
#                                     results.append((chinese_name, f"{conf:.2f}"))

#                                     # 手动绘制边界框和标签
#                                     xyxy = [int(i) for i in xyxy]
#                                     cv2.rectangle(im0, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors(c, True), 2)
#                                     cv2.putText(im0, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
#                                                 colors(c, True), 2)

#                                 # 打印结果
#                                 class_counts = {}
#                                 for cls_name, _ in results:
#                                     class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

#                                 # 修正中文输出
#                                 print_str_list = []
#                                 for k, v in class_counts.items():
#                                     if k == '人':
#                                         print_str_list.append(f"{v} {k}")
#                                     elif k == '公交车':
#                                         print_str_list.append(f"{v} {k}")
#                                     else:
#                                         print_str_list.append(f"{v} {k}{'s' if v > 1 and k not in ['人', '公交车'] else ''}")
#                                 print_str = ", ".join(print_str_list)

#                                 LOGGER.info(f"检测结果: {print_str}")
#                             else:
#                                 print_str = "未检测到目标"
#                                 LOGGER.info(print_str)

#                             # 保存结果到文件
#                             output_file = output_dir / (img_path.stem + ".txt")
#                             with open(output_file, 'w') as f:
#                                 f.write(print_str)
#                             LOGGER.info(f"检测结果已保存到: {output_file}")

#                             # 保存标注后的图片
#                             output_img_path = output_dir / img_path.name
#                             cv2.imwrite(str(output_img_path), im0)
#                             LOGGER.info(f"标注图片已保存到: {output_img_path}")
#                 except AssertionError as e:
#                     LOGGER.error(f"处理图片 {img_path.name} 时出错: {e}")
#                 except Exception as e:
#                     LOGGER.error(f"处理图片 {img_path.name} 时发生未知错误: {e}")

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



























## 需要roscore
# # # Ultralytics YOLOv5 🚀, AGPL - 3.0 license
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
#                 try:
#                     processed_files.add(img_path.name)  # 标记为已处理
#                     LOGGER.info(f"\n检测新图片: {img_path.name}")

#                     # 单图片处理流程
#                     dataset = LoadImages(str(img_path), img_size=imgsz, stride=stride, auto=pt)
#                     for path, im, im0s, _, _ in dataset:
#                         im = torch.from_numpy(im).to(device)
#                         im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
#                         im /= 255  # 0 - 255 to 0.0 - 1.0
#                         if len(im.shape) == 3:
#                             im = im[None]  # 扩展batch维度

#                         # 记录推理开始时间
#                         start_time = time.time()

#                         # 推理
#                         pred = model(im, augment=opt.augment, visualize=False)

#                         # 记录推理结束时间
#                         end_time = time.time()
#                         inference_time = end_time - start_time
#                         LOGGER.info(f"识别用时: {inference_time:.4f} 秒")

#                         # NMS
#                         pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres,
#                                                    opt.classes, opt.agnostic_nms, opt.max_det)

#                         # 处理检测结果
#                         results = []
#                         im0 = im0s.copy()
#                         for det in pred:
#                             if len(det):
#                                 det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

#                                 # 收集检测结果
#                                 for *xyxy, conf, cls in det:
#                                     c = int(cls)
#                                     original_name = names[c]
#                                     chinese_name = CHINESE_CLASS_NAMES.get(original_name, original_name)
#                                     label = f'{chinese_name} {conf:.2f}'
#                                     results.append((chinese_name, f"{conf:.2f}"))

#                                     # 手动绘制边界框和标签
#                                     xyxy = [int(i) for i in xyxy]
#                                     cv2.rectangle(im0, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors(c, True), 2)
#                                     cv2.putText(im0, label, (xyxy[0], xyxy[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
#                                                 colors(c, True), 2)

#                                 # 打印结果
#                                 class_counts = {}
#                                 for cls_name, _ in results:
#                                     class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

#                                 # 修正中文输出
#                                 print_str_list = []
#                                 for k, v in class_counts.items():
#                                     if k == '人':
#                                         print_str_list.append(f"{v} {k}")
#                                     elif k == '公交车':
#                                         print_str_list.append(f"{v} {k}")
#                                     else:
#                                         print_str_list.append(f"{v} {k}{'s' if v > 1 and k not in ['人', '公交车'] else ''}")
#                                 print_str = ", ".join(print_str_list)

#                                 LOGGER.info(f"检测结果: {print_str}")
#                             else:
#                                 print_str = "未检测到目标"
#                                 LOGGER.info(print_str)

#                             # 保存结果到文件
#                             output_file = output_dir / (img_path.stem + ".txt")
#                             with open(output_file, 'w') as f:
#                                 f.write(print_str)
#                             LOGGER.info(f"检测结果已保存到: {output_file}")

#                             # 保存标注后的图片
#                             output_img_path = output_dir / img_path.name
#                             cv2.imwrite(str(output_img_path), im0)
#                             LOGGER.info(f"标注图片已保存到: {output_img_path}")
#                 except AssertionError as e:
#                     LOGGER.error(f"处理图片 {img_path.name} 时出错: {e}")
#                 except Exception as e:
#                     LOGGER.error(f"处理图片 {img_path.name} 时发生未知错误: {e}")

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
