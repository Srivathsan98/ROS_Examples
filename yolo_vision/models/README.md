# Models Directory

Place your YOLOv8 ONNX model file here.

## Required File
- `yolov8n.onnx` - YOLOv8 nano model in ONNX format

## How to get the model
1. Download from Ultralytics: https://github.com/ultralytics/ultralytics
2. Convert to ONNX format using:
   ```python
   from ultralytics import YOLO
   model = YOLO('yolov8n.pt')
   model.export(format='onnx')
   ```

## Alternative models
You can also use other YOLOv8 variants:
- yolov8s.onnx (small)
- yolov8m.onnx (medium)
- yolov8l.onnx (large)
- yolov8x.onnx (extra large)

Just update the model filename in the code accordingly.
