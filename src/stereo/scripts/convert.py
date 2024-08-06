from ultralytics import YOLO

model = YOLO('/home/mrm/MRM-URC2024-NavStack/src/stereo/models/pt/bottle.pt')

model.export(format='engine')