import cv2

from mediapipe_controller import MP_Controller
import draw_landmarks


def main():
    # start video capture through webcam
    GAME_MODE = 2
    cap = cv2.VideoCapture(0)

    test_landmarker = MP_Controller(GAME_MODE)

    while True:
        # pull frame
        ret, frame = cap.read()
        # mirror frame
        frame = cv2.flip(frame, 1)

        test_landmarker.detect_async(frame, GAME_MODE)

        # draw landmarks on frame
        frame = draw_landmarks.draw_landmarks_on_hand(
            frame, test_landmarker.hand_result
        )

        if GAME_MODE == 2:
            frame = draw_landmarks.draw_landmarks_on_face(
                frame, test_landmarker.face_result
            )

        try:
            test_landmarker.get_index_tip_coordinates()
            if GAME_MODE == 2:
                test_landmarker.get_mouth_coordinates()
        except:
            pass

        # display image
        cv2.imshow("frame", frame)
        if cv2.waitKey(5) & 0xFF == 27:
            break

    # release everything
    test_landmarker.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
