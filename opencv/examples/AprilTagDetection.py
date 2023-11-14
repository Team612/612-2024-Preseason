import copy
import time
import argparse
import math
import cv2 as cv
from pupil_apriltags import Detector


actual_length = 6
pic_dist = 20
pixel_length = 300
focal_length = pixel_length*pic_dist/actual_length
#Get the arguments :)

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--families", type=str, default='tag16h5')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args


def main():
    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

    # カメラ準備 ###############################################################
    cap = cv.VideoCapture(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)

    # Detector準備 #############################################################
    at_detector = Detector(
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=refine_edges,
        decode_sharpening=decode_sharpening,
        debug=debug,
    )

    while True:
        start_time = time.time()

        # カメラキャプチャ #####################################################
        ret, image = cap.read()
        if not ret:
            break
        debug_image = copy.deepcopy(image)

        # 検出実施 #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags = at_detector.detect(
            image,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        # 描画 ################################################################
        debug_image = draw_tags(debug_image, tags)

        # キー処理(ESC：終了) #################################################
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

        # 画面反映 #############################################################
        cv.imshow('AprilTag Detect Demo', debug_image)
#ching chong cho
    cap.release()
    cv.destroyAllWindows()


def draw_tags(
    image,
    tags
):
    tag = 0
    fat_tyler = 0
    for i in tags:
        tyler = i.corners
        tylerarea = tyler[0][0]*tyler[1][1]+tyler[1][0]*tyler[2][1]+tyler[2][0]*tyler[3][1]+tyler[3][0]*tyler[0][1]
        tylerarea -= tyler[0][1]*tyler[1][0]+tyler[1][1]*tyler[2][0]+tyler[2][1]*tyler[3][0]+tyler[3][1]*tyler[0][0]
        tylerarea = abs(tylerarea)
        if tylerarea > fat_tyler:
            fat_tyler = tylerarea
            tag = i

    if tag == 0:
        return image
    tag_family = tag.tag_family
    tag_id = tag.tag_id
    center = tag.center
    corners = tag.corners

    center = (int(center[0]), int(center[1]))
    corner_01 = (int(corners[0][0]), int(corners[0][1]))
    corner_02 = (int(corners[1][0]), int(corners[1][1]))
    corner_03 = (int(corners[2][0]), int(corners[2][1]))
    corner_04 = (int(corners[3][0]), int(corners[3][1]))

    # 中心
    cv.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

    # 各辺
    cv.line(image, (corner_01[0], corner_01[1]),
            (corner_02[0], corner_02[1]), (255, 0, 0), 2)
    cv.line(image, (corner_02[0], corner_02[1]),
            (corner_03[0], corner_03[1]), (255, 0, 0), 2)
    cv.line(image, (corner_03[0], corner_03[1]),
            (corner_04[0], corner_04[1]), (0, 255, 0), 2)
    cv.line(image, (corner_04[0], corner_04[1]),
            (corner_01[0], corner_01[1]), (0, 255, 0), 2)

    # タグファミリー、タグID
    # cv.putText(image,
    #            str(tag_family) + ':' + str(tag_id),
    #            (corner_01[0], corner_01[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
    #            0.6, (0, 255, 0), 1, cv.LINE_AA)
    cv.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
            cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)

    cv.putText(image,
               "Distance:" + '{:.2f}'.format(focal_length*actual_length/math.sqrt(fat_tyler)) + "in",
               (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
               cv.LINE_AA)

    return image


if __name__ == '__main__':
    main()