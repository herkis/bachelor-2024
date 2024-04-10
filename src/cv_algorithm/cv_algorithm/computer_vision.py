import cv2
from ultralytics import YOLO
import time

def get_squares(results):
    outlist = []
    print(len(results[0].boxes.xyxy))
    for i in range(len(results[0].boxes.xyxy)):
        x = results[0].boxes.xyxy[i][0].item()
        y = results[0].boxes.xyxy[i][1].item()
        x2 = results[0].boxes.xyxy[i][2].item()
        y2 = results[0].boxes.xyxy[i][3].item()
        outlist.append([x, y, x2, y2])
    return outlist

def highlight_objects(frame, squares):
    for square in squares:
        top_left = (int(square[0]), int(square[1]))
        bottom_right = (int(square[2]), int(square[3]))
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), thickness=2)
    return frame

def main():
    # Output directory
    output_dir = "/home/SUMS/SUMS/images"

    # Initialize the YOLO model
    model_path = "/home/SUMS/SUMS/src/cv_algorithm/models/yolov8_weights.pt"
    model = YOLO(model_path)

    # Initialize webcam
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Error: Could not open webcam")
        exit()

    start_time = time.time()
    save_interval = 10  # seconds
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting ...")
            break

        # Perform detection (assuming direct frame processing is possible)
        results = model.predict(frame, conf=0.35)
        squares = get_squares(results)
        processed_frame = highlight_objects(frame.copy(), squares)

        current_time = time.time()
        if current_time - start_time >= save_interval:
            # Save processed frame with detections
            filename = f"{output_dir}/output_{frame_count}.jpg"
            cv2.imwrite(filename, processed_frame)
            print(f"Saved: {filename}")
            start_time = current_time
            frame_count += 1

    # When everything is done, release the capture
    cap.release()

if __name__ == '__main__':
    main()
