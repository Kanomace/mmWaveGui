import cv2

def record_video(output_file, duration, camera_index=0):
    # Open the camera
    cap = cv2.VideoCapture(camera_index)

    # Get the camera frame width and height
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Create a VideoWriter object to write the video
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, 20.0, (width, height))

    # Record video for the specified duration
    start_time = cv2.getTickCount()
    while (cv2.getTickCount() - start_time) / cv2.getTickFrequency() < duration:
        ret, frame = cap.read()  # Read a single frame
        if ret:
            out.write(frame)  # Write the current frame to the video file

            # Display the current frame
            cv2.imshow('Recording', frame)
            if cv2.waitKey(1) == ord('q'):  # Press 'q' to stop recording
                break
        else:
            break

    # Release resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()

# Test video recording
output_file = 'output.avi'  # Output file name
duration = 10  # Recording duration (seconds)

record_video(output_file, duration)
