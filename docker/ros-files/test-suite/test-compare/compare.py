import cv2
import imutils
import numpy as np

BG_SHADE = 0xcd
OCCUPIED_SHADE = 0x00
FREE_SHADE = 0xff
TEXT_SHADE = 0x00
BG_COLOR = (BG_SHADE, BG_SHADE, BG_SHADE)
OCCUPIED_COLOR = (OCCUPIED_SHADE, OCCUPIED_SHADE, OCCUPIED_SHADE)
FREE_COLOR = (FREE_SHADE, FREE_SHADE, FREE_SHADE)
TEXT_COLOR = (TEXT_SHADE, TEXT_SHADE, TEXT_SHADE)

class OccupationGrid:
    def __init__(self, image_filename):
        self.load(image_filename)

    def load(self, filename):
        self.original_image = cv2.imread(filename, cv2.IMREAD_UNCHANGED)

    def _resize(self, image, scale):
        width = np.ceil(image.shape[1] * scale)
        height = np.ceil(image.shape[0] * scale)
        new_size = (int(width), int(height))
        scaled_image = cv2.resize(image, new_size, interpolation = cv2.INTER_LINEAR_EXACT)
        return scaled_image

    def _rotate(self,
                image,
                angle,
                fill_color=BG_COLOR):
        height, width = image.shape[:2]
        image_center = (width/2, height/2)
        rotation_image = cv2.getRotationMatrix2D(image_center, angle, 1.)
        abs_cos = abs(rotation_image[0,0])
        abs_sin = abs(rotation_image[0,1])
        bound_w = int(height * abs_sin + width * abs_cos)
        bound_h = int(height * abs_cos + width * abs_sin)
        rotation_image[0, 2] += bound_w/2 - image_center[0]
        rotation_image[1, 2] += bound_h/2 - image_center[1]
        rotated_image = cv2.warpAffine(image,
                                       rotation_image,
                                       (bound_w, bound_h),
                                       borderMode=cv2.BORDER_CONSTANT,
                                       borderValue=fill_color)
        return rotated_image

    def _pad_to_square(self,
                       image,
                       new_size,
                       fill_color=BG_COLOR):
        width = np.ceil(image.shape[1])
        height = np.ceil(image.shape[0])
        new_width = new_height = new_size

        vertical_padding = (new_height - height)
        horizontal_padding = (new_width - width)
        top = np.ceil(vertical_padding / 2)
        down = np.floor(vertical_padding / 2)
        left = np.ceil(horizontal_padding / 2)
        right = np.floor(horizontal_padding / 2)

        padded_image = cv2.copyMakeBorder(image,
                                          int(top),
                                          int(down),
                                          int(left),
                                          int(right),
                                          borderType=cv2.BORDER_CONSTANT,
                                          value=fill_color)
        return padded_image

    def calc_diagonal(self, image):
        height, width = image.shape[:2]
        diagonal_length = np.sqrt(pow(height, 2) + pow(width, 2))
        return np.ceil(diagonal_length)

    def _crop_to_content(self, image):
        thr = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY_INV)[1]
        bbox = cv2.boundingRect(thr)
        x, y, w, h = bbox
        assert w > 0
        assert h > 0
        cropped = image[y:y+h, x:x+w]
        return cropped

    def _haystack_image(self, angle):
        ret = self._crop_to_content(self.original_image)

        # Diagonal is the maximum width or height dimension
        # after rotation, hence it is calculated from cropped
        # and scaled image (not yet rotated because rotation
        # affects width and height).
        new_padded_size = self.calc_diagonal(ret) * 3
        ret = self._rotate(ret, angle)

        # The intention here is to put the image in the middle
        # and have enough space to fit the same sized image at
        # any angle, in any position relative to it.
        ret = self._pad_to_square(ret, new_padded_size, BG_COLOR)
        return ret

    def _needle_image(self, scale):
        ret = self._crop_to_content(self.original_image)
        ret = self._resize(ret, scale)
        return ret

    def _put_text(self, image, text):
        ret = image.copy()
        h, w = ret.shape[:2]
        scale = h / 1200
        thickness = int(np.ceil(h / 1000))

        x0 = int(w * 0.05)
        y0 = int(0.75 * h)
        for i, line in enumerate(text.split('\n')):
            y = int(np.ceil(i * 40 * scale))
            cv2.putText(ret,
                        line,
                        (x0, y0 + y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        scale,
                        TEXT_COLOR,
                        thickness,
                        cv2.FILLED)
        return ret

    def preview(self,
                other_grid,
                pos,
                angle,
                scale,
                text,
                wait_time = 1):
        haystack = self._haystack_image(angle).copy()
        needle = other_grid._needle_image(scale).copy()
        needle_h, needle_w = needle.shape[:2]
        pos_x, pos_y = pos
        alpha = 0.5

        haystack[pos_y:pos_y + needle_h, pos_x:pos_x + needle_w] = \
            haystack[pos_y:pos_y + needle_h, pos_x:pos_x + needle_w] * alpha + \
            needle * (1 - alpha)


        haystack = self._put_text(haystack, text)
        resized = cv2.resize(haystack, (1024, 768))
        cv2.imshow('preview', resized)
        cv2.waitKey(wait_time)

    def match_template(self,
                       other_grid,
                       scale_step = 0.1,
                       angle_step = 6,
                       debug_preview = False):
        assert(1 / scale_step % 1 == 0)
        assert(360 / angle_step % 1 == 0)
        image = self
        template = other_grid

        # ratio of two diameters, required to normalize template size
        # with regard to image
        image_to_template_cropped_ratio = \
            image.calc_diagonal(image._needle_image(1)) / \
            template.calc_diagonal(template._needle_image(1))

        max_val = 0
        max_loc = None
        max_angle = None
        max_scale = None
        max_updated_scale = None
        for angle in np.linspace(0, (360 - angle_step), int(360 / angle_step)):
            haystack = image._haystack_image(angle)

            for scale in np.linspace(scale_step, 1, int(1 / scale_step)):
                updated_scale = image_to_template_cropped_ratio * scale
                needle = template._needle_image(updated_scale)

                result = cv2.matchTemplate(haystack, needle, cv2.TM_CCOEFF_NORMED)
                _, curr_max_val, _, curr_max_loc = cv2.minMaxLoc(result)

                if curr_max_val > max_val:
                    max_val = curr_max_val
                    max_loc = curr_max_loc
                    max_angle = angle
                    max_scale = scale
                    max_updated_scale = updated_scale

                text = f'current angle:         {round(angle, 3)}\n' \
                       f'current scale:         {round(scale, 3)}\n' \
                       f'current updated scale: {round(updated_scale, 3)}\n' \
                       f'current location:      ({curr_max_loc[0]}, {curr_max_loc[1]})\n' \
                       f'current score:         {round(curr_max_val, 3)}\n' \
                       f'current max score:     {round(max_val, 3)}\n'
                if debug_preview:
                    image.preview(template,
                                  curr_max_loc,
                                  angle,
                                  updated_scale,
                                  text)
                else:
                    print(f'---\n{text}')

        return (max_val, max_loc, max_angle, max_scale, max_updated_scale)


def compare(a, b):
    max_val, max_loc, max_angle, max_scale, max_updated_scale = \
        a.match_template(b, debug_preview = True)

    a.preview(b,
              max_loc,
              max_angle,
              max_updated_scale,
              f'maximum angle:      {round(max_angle, 3)}\n'
              f'maximum scale:      {round(max_updated_scale, 3)}\n'
              f'maximum location:   ({max_loc[0]}, {max_loc[1]})\n'
              f'maximum score:      {round(max_val, 3)}',
              wait_time = 0)

    return (max_val, max_loc, max_angle, max_scale, max_updated_scale)


a = OccupationGrid('0.01.pgm')
b = OccupationGrid('0.13.pgm')
debug = True
visual_summary = True

print('first pass')
max_val, max_loc, max_angle, max_scale, max_updated_scale = \
    a.match_template(b, debug_preview = debug)

print('second pass')
max_val_2, max_loc_2, max_angle_2, max_scale_2, max_updated_scale_2 = \
    b.match_template(a, debug_preview = debug)

if max_val > max_val_2:
    print(f'SCORE: {round(max_val, 3)}')
    if visual_summary:
        a.preview(b,
                  max_loc,
                  max_angle,
                  max_updated_scale,
                  f'maximum angle:      {round(max_angle, 3)}\n'
                  f'maximum scale:      {round(max_updated_scale, 3)}\n'
                  f'maximum location:   ({max_loc[0]}, {max_loc[1]})\n'
                  f'maximum score:      {round(max_val, 3)}',
                  wait_time = 0)
else:
    print(f'SCORE: {round(max_val_2, 3)}')
    if visual_summary:
        b.preview(a,
                  max_loc_2,
                  max_angle_2,
                  max_updated_scale_2,
                  f'maximum angle:      {round(max_angle_2, 3)}\n'
                  f'maximum scale:      {round(max_updated_scale_2, 3)}\n'
                  f'maximum location:   ({max_loc_2[0]}, {max_loc_2[1]})\n'
                  f'maximum score:      {round(max_val_2, 3)}',
                  wait_time = 0)

#TODO make this script non-interactive
