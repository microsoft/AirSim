/*
*****************************************************************************
*                                                                           *
*                          Platform Independent                             *
*                    Bitmap Image Reader Writer Library                     *
*                                                                           *
* Author: Arash Partow - 2002                                               *
* URL: http://partow.net/programming/bitmap/index.html                      *
*                                                                           *
* Note: This library only supports 24-bits per pixel bitmap format files.   *
*                                                                           *
* Copyright notice:                                                         *
* Free use of the Platform Independent Bitmap Image Reader Writer Library   *
* is permitted under the guidelines and in accordance with the most current *
* version of the MIT License.                                               *
* http://www.opensource.org/licenses/MIT                                    *
*                                                                           *
*****************************************************************************
*/

#ifndef INCLUDE_BITMAP_IMAGE_HPP
#define INCLUDE_BITMAP_IMAGE_HPP

// clang-format off

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace common_utils { namespace bmp  {

class bitmap_image
{
public:

    enum channel_mode {
        rgb_mode = 0,
        bgr_mode = 1
    };

    enum color_plane {
        blue_plane = 0,
        green_plane = 1,
        red_plane = 2
    };

    struct rgb_t
    {
        unsigned char   red;
        unsigned char green;
        unsigned char  blue;
    };

    bitmap_image()
        : file_name_(""),
        width_(0),
        height_(0),
        row_increment_(0),
        bytes_per_pixel_(3),
        channel_mode_(bgr_mode)
    {}

    bitmap_image(const std::string& filename)
        : file_name_(filename),
        width_(0),
        height_(0),
        row_increment_(0),
        bytes_per_pixel_(0),
        channel_mode_(bgr_mode)
    {
        load_bitmap();
    }

    bitmap_image(const unsigned int width, const unsigned int height)
        : file_name_(""),
        width_(width),
        height_(height),
        row_increment_(0),
        bytes_per_pixel_(3),
        channel_mode_(bgr_mode)
    {
        create_bitmap();
    }

    bitmap_image(const bitmap_image& image)
        : file_name_(image.file_name_),
        width_(image.width_),
        height_(image.height_),
        row_increment_(0),
        bytes_per_pixel_(3),
        channel_mode_(bgr_mode)
    {
        create_bitmap();
        data_ = image.data_;
    }

    bitmap_image& operator=(const bitmap_image& image)
    {
        if (this != &image)
        {
            file_name_ = image.file_name_;
            bytes_per_pixel_ = image.bytes_per_pixel_;
            width_ = image.width_;
            height_ = image.height_;
            row_increment_ = 0;
            channel_mode_ = image.channel_mode_;
            create_bitmap();
            data_ = image.data_;
        }

        return *this;
    }

    inline bool operator!()
    {
        return (data_.size() == 0) ||
            (width_ == 0) ||
            (height_ == 0) ||
            (row_increment_ == 0);
    }

    inline void clear(const unsigned char v = 0x00)
    {
        std::fill(data_.begin(), data_.end(), v);
    }

    inline unsigned char red_channel(const unsigned int x, const unsigned int y) const
    {
        return data_[(y * row_increment_) + (x * bytes_per_pixel_ + 2)];
    }

    inline unsigned char green_channel(const unsigned int x, const unsigned int y) const
    {
        return data_[(y * row_increment_) + (x * bytes_per_pixel_ + 1)];
    }

    inline unsigned char blue_channel(const unsigned int x, const unsigned int y) const
    {
        return data_[(y * row_increment_) + (x * bytes_per_pixel_ + 0)];
    }

    inline void red_channel(const unsigned int x, const unsigned int y, const unsigned char value)
    {
        data_[(y * row_increment_) + (x * bytes_per_pixel_ + 2)] = value;
    }

    inline void green_channel(const unsigned int x, const unsigned int y, const unsigned char value)
    {
        data_[(y * row_increment_) + (x * bytes_per_pixel_ + 1)] = value;
    }

    inline void blue_channel(const unsigned int x, const unsigned int y, const unsigned char value)
    {
        data_[(y * row_increment_) + (x * bytes_per_pixel_ + 0)] = value;
    }

    inline unsigned char* row(unsigned int row_index) const
    {
        return const_cast<unsigned char*>(&data_[(row_index * row_increment_)]);
    }

    inline void get_pixel(const unsigned int x, const unsigned int y,
        unsigned char& red,
        unsigned char& green,
        unsigned char& blue) const
    {
        const unsigned int y_offset = y * row_increment_;
        const unsigned int x_offset = x * bytes_per_pixel_;
        const unsigned int offset = y_offset + x_offset;

        blue = data_[offset + 0];
        green = data_[offset + 1];
        red = data_[offset + 2];
    }

    template <typename RGB>
    inline void get_pixel(const unsigned int x, const unsigned int y, RGB& colour) const
    {
        get_pixel(x, y, colour.red, colour.green, colour.blue);
    }

    inline rgb_t get_pixel(const unsigned int x, const unsigned int y) const
    {
        rgb_t colour;
        get_pixel(x, y, colour.red, colour.green, colour.blue);
        return colour;
    }

    inline void set_pixel(const unsigned int x, const unsigned int y,
        const unsigned char red,
        const unsigned char green,
        const unsigned char blue)
    {
        const unsigned int y_offset = y * row_increment_;
        const unsigned int x_offset = x * bytes_per_pixel_;
        const unsigned int offset = y_offset + x_offset;

        data_[offset + 0] = blue;
        data_[offset + 1] = green;
        data_[offset + 2] = red;
    }

    template <typename RGB>
    inline void set_pixel(const unsigned int x, const unsigned int y, const RGB& colour)
    {
        set_pixel(x, y, colour.red, colour.green, colour.blue);
    }

    inline bool copy_from(const bitmap_image& image)
    {
        if (
            (image.height_ != height_) ||
            (image.width_ != width_)
            )
        {
            return false;
        }

        data_ = image.data_;

        return true;
    }

    inline bool copy_from(const bitmap_image& source_image,
        const unsigned int& x_offset,
        const unsigned int& y_offset)
    {
        if ((x_offset + source_image.width_) > width_) { return false; }
        if ((y_offset + source_image.height_) > height_) { return false; }

        for (unsigned int y = 0; y < source_image.height_; ++y)
        {
            unsigned char* itr1 = row(y + y_offset) + x_offset * bytes_per_pixel_;
            const unsigned char* itr2 = source_image.row(y);
            const unsigned char* itr2_end = itr2 + source_image.width_ * bytes_per_pixel_;

            std::copy(itr2, itr2_end, itr1);
        }

        return true;
    }

    inline bool region(const unsigned int& x,
        const unsigned int& y,
        const unsigned int& width,
        const unsigned int& height,
        bitmap_image& dest_image) const
    {
        if ((x + width) > width_) { return false; }
        if ((y + height) > height_) { return false; }

        if (
            (dest_image.width_  < width_) ||
            (dest_image.height_ < height_)
            )
        {
            dest_image.setwidth_height(width, height);
        }

        for (unsigned int r = 0; r < height; ++r)
        {
            unsigned char* itr1 = row(r + y) + x * bytes_per_pixel_;
            unsigned char* itr1_end = itr1 + (width * bytes_per_pixel_);
            unsigned char* itr2 = dest_image.row(r);

            std::copy(itr1, itr1_end, itr2);
        }

        return true;
    }

    inline bool roi_from_center(const unsigned int& cx,
        const unsigned int& cy,
        const unsigned int& width,
        const unsigned int& height,
        bitmap_image& dest_image) const
    {
        return region(cx - (width / 2), cy - (height / 2),
            width, height,
            dest_image);
    }

    inline bool set_region(const unsigned int&  x,
        const unsigned int&  y,
        const unsigned int&  width,
        const unsigned int&  height,
        const unsigned char& value)
    {
        if ((x + width) > width_) { return false; }
        if ((y + height) > height_) { return false; }

        for (unsigned int r = 0; r < height; ++r)
        {
            unsigned char* itr = row(r + y) + x * bytes_per_pixel_;
            unsigned char* itr_end = itr + (width * bytes_per_pixel_);

            std::fill(itr, itr_end, value);
        }

        return true;
    }

    inline bool set_region(const unsigned int&  x,
        const unsigned int&  y,
        const unsigned int&  width,
        const unsigned int&  height,
        const color_plane    color,
        const unsigned char& value)
    {
        if ((x + width) > width_) { return false; }
        if ((y + height) > height_) { return false; }

        const unsigned int color_plane_offset = offset(color);

        for (unsigned int r = 0; r < height; ++r)
        {
            unsigned char* itr = row(r + y) + x * bytes_per_pixel_ + color_plane_offset;
            unsigned char* itr_end = itr + (width * bytes_per_pixel_);

            while (itr != itr_end)
            {
                *itr = value;
                itr += bytes_per_pixel_;
            }
        }

        return true;
    }

    inline bool set_region(const unsigned int&  x,
        const unsigned int&  y,
        const unsigned int&  width,
        const unsigned int&  height,
        const unsigned char& red,
        const unsigned char& green,
        const unsigned char& blue)
    {
        if ((x + width) > width_) { return false; }
        if ((y + height) > height_) { return false; }

        for (unsigned int r = 0; r < height; ++r)
        {
            unsigned char* itr = row(r + y) + x * bytes_per_pixel_;
            unsigned char* itr_end = itr + (width * bytes_per_pixel_);

            while (itr != itr_end)
            {
                *(itr++) = blue;
                *(itr++) = green;
                *(itr++) = red;
            }
        }

        return true;
    }

    void reflective_image(bitmap_image& image, const bool include_diagnols = false)
    {
        image.setwidth_height(3 * width_, 3 * height_, true);

        image.copy_from(*this, width_, height_);

        vertical_flip();

        image.copy_from(*this, width_, 0);
        image.copy_from(*this, width_, 2 * height_);

        vertical_flip();
        horizontal_flip();

        image.copy_from(*this, 0, height_);
        image.copy_from(*this, 2 * width_, height_);

        horizontal_flip();

        if (include_diagnols)
        {
            bitmap_image tile = *this;

            tile.vertical_flip();
            tile.horizontal_flip();

            image.copy_from(tile, 0, 0);
            image.copy_from(tile, 2 * width_, 0);
            image.copy_from(tile, 2 * width_, 2 * height_);
            image.copy_from(tile, 0, 2 * height_);
        }
    }

    inline unsigned int width() const
    {
        return width_;
    }

    inline unsigned int height() const
    {
        return height_;
    }

    inline unsigned int bytes_per_pixel() const
    {
        return bytes_per_pixel_;
    }

    inline unsigned int pixel_count() const
    {
        return width_ * height_;
    }

    inline void setwidth_height(const unsigned int width,
        const unsigned int height,
        const bool clear = false)
    {
        data_.clear();
        width_ = width;
        height_ = height;

        create_bitmap();

        if (clear)
        {
            std::fill(data_.begin(), data_.end(), static_cast<unsigned char>(0x00));
        }
    }

    void save_image(const std::string& file_name) const
    {
        std::ofstream stream(file_name.c_str(), std::ios::binary);

        if (!stream)
        {
            std::cerr << "bitmap_image::save_image(): Error - Could not open file " << file_name << " for writing!" << std::endl;
            return;
        }

        bitmap_information_header bih;

        bih.width = width_;
        bih.height = height_;
        bih.bit_count = static_cast<unsigned short>(bytes_per_pixel_ << 3);
        bih.clr_important = 0;
        bih.clr_used = 0;
        bih.compression = 0;
        bih.planes = 1;
        bih.size = bih.struct_size();
        bih.x_pels_per_meter = 0;
        bih.y_pels_per_meter = 0;
        bih.size_image = (((bih.width * bytes_per_pixel_) + 3) & 0x0000FFFC) * bih.height;

        bitmap_file_header bfh;

        bfh.type = 19778;
        bfh.size = bfh.struct_size() + bih.struct_size() + bih.size_image;
        bfh.reserved1 = 0;
        bfh.reserved2 = 0;
        bfh.off_bits = bih.struct_size() + bfh.struct_size();

        write_bfh(stream, bfh);
        write_bih(stream, bih);

        unsigned int padding = (4 - ((3 * width_) % 4)) % 4;
        char padding_data[4] = { 0x00, 0x00, 0x00, 0x00 };

        for (unsigned int i = 0; i < height_; ++i)
        {
            const unsigned char* data_ptr = &data_[(row_increment_ * (height_ - i - 1))];

            stream.write(reinterpret_cast<const char*>(data_ptr), sizeof(unsigned char) * bytes_per_pixel_ * width_);
            stream.write(padding_data, padding);
        }

        stream.close();
    }

    inline void set_all_ith_bits_low(const unsigned int bitr_index)
    {
        unsigned char mask = static_cast<unsigned char>(~(1 << bitr_index));

        for (unsigned char* itr = data(); itr != end(); ++itr)
        {
            *itr &= mask;
        }
    }

    inline void set_all_ith_bits_high(const unsigned int bitr_index)
    {
        unsigned char mask = static_cast<unsigned char>(1 << bitr_index);

        for (unsigned char* itr = data(); itr != end(); ++itr)
        {
            *itr |= mask;
        }
    }

    inline void set_all_ith_channels(const unsigned int& channel, const unsigned char& value)
    {
        for (unsigned char* itr = (data() + channel); itr < end(); itr += bytes_per_pixel_)
        {
            *itr = value;
        }
    }

    inline void set_channel(const color_plane color, const unsigned char& value)
    {
        for (unsigned char* itr = (data() + offset(color)); itr < end(); itr += bytes_per_pixel_)
        {
            *itr = value;
        }
    }

    inline void ror_channel(const color_plane color, const unsigned int& ror)
    {
        for (unsigned char* itr = (data() + offset(color)); itr < end(); itr += bytes_per_pixel_)
        {
            *itr = static_cast<unsigned char>(((*itr) >> ror) | ((*itr) << (8 - ror)));
        }
    }

    inline void set_all_channels(const unsigned char& value)
    {
        for (unsigned char* itr = data(); itr < end(); )
        {
            *(itr++) = value;
        }
    }

    inline void set_all_channels(const unsigned char& r_value,
        const unsigned char& g_value,
        const unsigned char& b_value)
    {
        for (unsigned char* itr = (data() + 0); itr < end(); itr += bytes_per_pixel_)
        {
            *(itr + 0) = b_value;
            *(itr + 1) = g_value;
            *(itr + 2) = r_value;
        }
    }

    inline void invert_color_planes()
    {
        for (unsigned char* itr = data(); itr < end(); *itr = ~(*itr), ++itr);
    }

    inline void add_to_color_plane(const color_plane color, const unsigned char& value)
    {
        for (unsigned char* itr = (data() + offset(color)); itr < end(); itr += bytes_per_pixel_)
        {
            (*itr) += value;
        }
    }

    inline void convert_to_grayscale()
    {
        double r_scaler = 0.299;
        double g_scaler = 0.587;
        double b_scaler = 0.114;

        if (rgb_mode == channel_mode_)
        {
            std::swap(r_scaler, b_scaler);
        }

        for (unsigned char* itr = data(); itr < end(); )
        {
            unsigned char gray_value = static_cast<unsigned char>
                (
                (r_scaler * (*(itr + 2))) +
                    (g_scaler * (*(itr + 1))) +
                    (b_scaler * (*(itr + 0)))
                    );

            *(itr++) = gray_value;
            *(itr++) = gray_value;
            *(itr++) = gray_value;
        }
    }

    inline const unsigned char* data() const
    {
        return data_.data();
    }

    inline unsigned char* data()
    {
        return const_cast<unsigned char*>(data_.data());
    }

    inline void bgr_to_rgb()
    {
        if ((bgr_mode == channel_mode_) && (3 == bytes_per_pixel_))
        {
            reverse_channels();
            channel_mode_ = rgb_mode;
        }
    }

    inline void rgb_to_bgr()
    {
        if ((rgb_mode == channel_mode_) && (3 == bytes_per_pixel_))
        {
            reverse_channels();
            channel_mode_ = bgr_mode;
        }
    }

    inline void reverse()
    {
        unsigned char* itr1 = data();
        unsigned char* itr2 = end() - bytes_per_pixel_;

        while (itr1 < itr2)
        {
            for (std::size_t i = 0; i < bytes_per_pixel_; ++i)
            {
                unsigned char* citr1 = itr1 + i;
                unsigned char* citr2 = itr2 + i;

                std::swap(*citr1, *citr2);
            }

            itr1 += bytes_per_pixel_;
            itr2 -= bytes_per_pixel_;
        }
    }

    inline void horizontal_flip()
    {
        for (unsigned int y = 0; y < height_; ++y)
        {
            unsigned char* itr1 = row(y);
            unsigned char* itr2 = itr1 + row_increment_ - bytes_per_pixel_;

            while (itr1 < itr2)
            {
                for (unsigned int i = 0; i < bytes_per_pixel_; ++i)
                {
                    unsigned char* p1 = (itr1 + i);
                    unsigned char* p2 = (itr2 + i);

                    std::swap(*p1, *p2);
                }

                itr1 += bytes_per_pixel_;
                itr2 -= bytes_per_pixel_;
            }
        }
    }

    inline void vertical_flip()
    {
        for (unsigned int y = 0; y < (height_ / 2); ++y)
        {
            unsigned char* itr1 = row(y);
            unsigned char* itr2 = row(height_ - y - 1);

            for (std::size_t x = 0; x < row_increment_; ++x)
            {
                std::swap(*(itr1 + x), *(itr2 + x));
            }
        }
    }

    inline void export_color_plane(const color_plane color, unsigned char* image)
    {
        for (unsigned char* itr = (data() + offset(color)); itr < end(); ++image, itr += bytes_per_pixel_)
        {
            (*image) = (*itr);
        }
    }

    inline void export_color_plane(const color_plane color, bitmap_image& image)
    {
        if (
            (width_ != image.width_) ||
            (height_ != image.height_)
            )
        {
            image.setwidth_height(width_, height_);
        }

        image.clear();

        unsigned char* itr1 = (data() + offset(color));
        unsigned char* itr1_end = end();
        unsigned char* itr2 = (image.data() + offset(color));

        while (itr1 < itr1_end)
        {
            (*itr2) = (*itr1);

            itr1 += bytes_per_pixel_;
            itr2 += bytes_per_pixel_;
        }
    }

    inline void export_response_image(const color_plane color, double* response_image)
    {
        double* resp_itr = response_image;

        for (unsigned char* itr = (data() + offset(color)); itr < end(); ++response_image, itr += bytes_per_pixel_)
        {
            *(resp_itr++) = (1.0 * (*itr)) / 256.0;
        }
    }

    inline void export_gray_scale_response_image(double* response_image) const
    {
        double* resp_itr = response_image;

        for (const unsigned char* itr = data(); itr < end(); itr += bytes_per_pixel_)
        {
            unsigned char gray_value = static_cast<unsigned char>
                (
                (0.299 * (*(itr + 2))) +
                    (0.587 * (*(itr + 1))) +
                    (0.114 * (*(itr + 0)))
                    );

            *(resp_itr++) = (1.0 * gray_value) / 256.0;
        }
    }

    inline void export_rgb(double* red, double* green, double* blue) const
    {
        if (bgr_mode != channel_mode_)
            return;

        for (const unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            (*blue) = (1.0 * (*(itr++))) / 256.0;
            (*green) = (1.0 * (*(itr++))) / 256.0;
            (*red) = (1.0 * (*(itr++))) / 256.0;
        }
    }

    inline void export_rgb(float* red, float* green, float* blue) const
    {
        if (bgr_mode != channel_mode_)
            return;

        for (const unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            (*blue) = (1.0f * (*(itr++))) / 256.0f;
            (*green) = (1.0f * (*(itr++))) / 256.0f;
            (*red) = (1.0f * (*(itr++))) / 256.0f;
        }
    }

    inline void export_rgb(unsigned char* red, unsigned char* green, unsigned char* blue) const
    {
        if (bgr_mode != channel_mode_)
            return;

        for (const unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            (*blue) = *(itr++);
            (*green) = *(itr++);
            (*red) = *(itr++);
        }
    }

    inline void export_ycbcr(double* y, double* cb, double* cr) const
    {
        if (bgr_mode != channel_mode_)
            return;

        for (const unsigned char* itr = data(); itr < end(); ++y, ++cb, ++cr)
        {
            const double blue = (1.0 * (*(itr++)));
            const double green = (1.0 * (*(itr++)));
            const double red = (1.0 * (*(itr++)));

            (*y) = clamp<double>(16.0 + (1.0 / 256.0) * (65.738 * red + 129.057 * green + 25.064 * blue), 1.0, 254);
            (*cb) = clamp<double>(128.0 + (1.0 / 256.0) * (-37.945 * red - 74.494 * green + 112.439 * blue), 1.0, 254);
            (*cr) = clamp<double>(128.0 + (1.0 / 256.0) * (112.439 * red - 94.154 * green - 18.285 * blue), 1.0, 254);
        }
    }

    inline void export_rgb_normal(double* red, double* green, double* blue) const
    {
        if (bgr_mode != channel_mode_)
            return;

        for (const unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            (*blue) = (1.0 * (*(itr++)));
            (*green) = (1.0 * (*(itr++)));
            (*red) = (1.0 * (*(itr++)));
        }
    }

    inline void export_rgb_normal(float* red, float* green, float* blue) const
    {
        if (bgr_mode != channel_mode_)
            return;

        for (const unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            (*blue) = (1.0f * (*(itr++)));
            (*green) = (1.0f * (*(itr++)));
            (*red) = (1.0f * (*(itr++)));
        }
    }

    inline void import_rgb(double* red, double* green, double* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = static_cast<unsigned char>(256.0 * (*blue));
            *(itr++) = static_cast<unsigned char>(256.0 * (*green));
            *(itr++) = static_cast<unsigned char>(256.0 * (*red));
        }
    }

    inline void import_rgb(float* red, float* green, float* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = static_cast<unsigned char>(256.0f * (*blue));
            *(itr++) = static_cast<unsigned char>(256.0f * (*green));
            *(itr++) = static_cast<unsigned char>(256.0f * (*red));
        }
    }

    inline void import_rgb(unsigned char* red, unsigned char* green, unsigned char* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = (*blue);
            *(itr++) = (*green);
            *(itr++) = (*red);
        }
    }

    inline void import_ycbcr(double* y, double* cb, double* cr)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++y, ++cb, ++cr)
        {
            double y_ = (*y);
            double cb_ = (*cb);
            double cr_ = (*cr);

            *(itr++) = static_cast<unsigned char>(clamp((298.082 * y_ + 516.412 * cb_) / 256.0 - 276.836, 0.0, 255.0));
            *(itr++) = static_cast<unsigned char>(clamp((298.082 * y_ - 100.291 * cb_ - 208.120 * cr_) / 256.0 + 135.576, 0.0, 255.0));
            *(itr++) = static_cast<unsigned char>(clamp((298.082 * y_ + 408.583 * cr_) / 256.0 - 222.921, 0.0, 255.0));
        }
    }

    inline void import_gray_scale_clamped(double* gray)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++gray)
        {
            unsigned char c = static_cast<unsigned char>(clamp<double>(256.0 * (*gray), 0.0, 255.0));

            *(itr + 0) = c;
            *(itr + 1) = c;
            *(itr + 2) = c;

            itr += 3;
        }
    }

    inline void import_rgb_clamped(double* red, double* green, double* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = static_cast<unsigned char>(clamp<double>(256.0 * (*blue), 0.0, 255.0));
            *(itr++) = static_cast<unsigned char>(clamp<double>(256.0 * (*green), 0.0, 255.0));
            *(itr++) = static_cast<unsigned char>(clamp<double>(256.0 * (*red), 0.0, 255.0));
        }
    }

    inline void import_rgb_clamped(float* red, float* green, float* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = static_cast<unsigned char>(clamp<double>(256.0f * (*blue), 0.0, 255.0));
            *(itr++) = static_cast<unsigned char>(clamp<double>(256.0f * (*green), 0.0, 255.0));
            *(itr++) = static_cast<unsigned char>(clamp<double>(256.0f * (*red), 0.0, 255.0));
        }
    }

    inline void import_rgb_normal(double* red, double* green, double* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = static_cast<unsigned char>(*blue);
            *(itr++) = static_cast<unsigned char>(*green);
            *(itr++) = static_cast<unsigned char>(*red);
        }
    }

    inline void import_rgb_normal(float* red, float* green, float* blue)
    {
        if (bgr_mode != channel_mode_)
            return;

        for (unsigned char* itr = data(); itr < end(); ++red, ++green, ++blue)
        {
            *(itr++) = static_cast<unsigned char>(*blue);
            *(itr++) = static_cast<unsigned char>(*green);
            *(itr++) = static_cast<unsigned char>(*red);
        }
    }

    inline void subsample(bitmap_image& dest) const
    {
        /*
        Half sub-sample of original image.
        */
        unsigned int w = 0;
        unsigned int h = 0;

        bool odd_width = false;
        bool odd_height = false;

        if (0 == (width_ % 2))
            w = width_ / 2;
        else
        {
            w = 1 + (width_ / 2);
            odd_width = true;
        }

        if (0 == (height_ % 2))
            h = height_ / 2;
        else
        {
            h = 1 + (height_ / 2);
            odd_height = true;
        }

        unsigned int horizontal_upper = (odd_width) ? (w - 1) : w;
        unsigned int vertical_upper = (odd_height) ? (h - 1) : h;

        dest.setwidth_height(w, h);
        dest.clear();

        unsigned char* s_itr[3];
        const unsigned char*  itr1[3];
        const unsigned char*  itr2[3];

        s_itr[0] = dest.data() + 0;
        s_itr[1] = dest.data() + 1;
        s_itr[2] = dest.data() + 2;

        itr1[0] = data() + 0;
        itr1[1] = data() + 1;
        itr1[2] = data() + 2;

        itr2[0] = data() + row_increment_ + 0;
        itr2[1] = data() + row_increment_ + 1;
        itr2[2] = data() + row_increment_ + 2;

        unsigned int total = 0;

        for (unsigned int j = 0; j < vertical_upper; ++j)
        {
            for (unsigned int i = 0; i < horizontal_upper; ++i)
            {
                for (unsigned int k = 0; k < bytes_per_pixel_; s_itr[k] += bytes_per_pixel_, ++k)
                {
                    total = 0;
                    total += *(itr1[k]);
                    total += *(itr1[k]);
                    total += *(itr2[k]);
                    total += *(itr2[k]);

                    itr1[k] += bytes_per_pixel_;
                    itr1[k] += bytes_per_pixel_;
                    itr2[k] += bytes_per_pixel_;
                    itr2[k] += bytes_per_pixel_;

                    *(s_itr[k]) = static_cast<unsigned char>(total >> 2);
                }
            }

            if (odd_width)
            {
                for (unsigned int k = 0; k < bytes_per_pixel_; s_itr[k] += bytes_per_pixel_, ++k)
                {
                    total = 0;
                    total += *(itr1[k]);
                    total += *(itr2[k]);

                    itr1[k] += bytes_per_pixel_;
                    itr2[k] += bytes_per_pixel_;

                    *(s_itr[k]) = static_cast<unsigned char>(total >> 1);
                }
            }

            for (unsigned int k = 0; k < bytes_per_pixel_; ++k)
            {
                itr1[k] += row_increment_;
            }

            if (j != (vertical_upper - 1))
            {
                for (unsigned int k = 0; k < bytes_per_pixel_; ++k)
                {
                    itr2[k] += row_increment_;
                }
            }
        }

        if (odd_height)
        {
            for (unsigned int i = 0; i < horizontal_upper; ++i)
            {
                for (unsigned int k = 0; k < bytes_per_pixel_; s_itr[k] += bytes_per_pixel_, ++k)
                {
                    total = 0;
                    total += *(itr1[k]);
                    total += *(itr2[k]);

                    itr1[k] += bytes_per_pixel_;
                    itr2[k] += bytes_per_pixel_;

                    *(s_itr[k]) = static_cast<unsigned char>(total >> 1);
                }
            }

            if (odd_width)
            {
                for (unsigned int k = 0; k < bytes_per_pixel_; ++k)
                {
                    (*(s_itr[k])) = *(itr1[k]);
                }
            }
        }
    }

    inline void upsample(bitmap_image& dest) const
    {
        /*
        2x up-sample of original image.
        */

        dest.setwidth_height(2 * width_, 2 * height_);
        dest.clear();

        const unsigned char* s_itr[3];
        unsigned char*  itr1[3];
        unsigned char*  itr2[3];

        s_itr[0] = data() + 0;
        s_itr[1] = data() + 1;
        s_itr[2] = data() + 2;

        itr1[0] = dest.data() + 0;
        itr1[1] = dest.data() + 1;
        itr1[2] = dest.data() + 2;

        itr2[0] = dest.data() + dest.row_increment_ + 0;
        itr2[1] = dest.data() + dest.row_increment_ + 1;
        itr2[2] = dest.data() + dest.row_increment_ + 2;

        for (unsigned int j = 0; j < height_; ++j)
        {
            for (unsigned int i = 0; i < width_; ++i)
            {
                for (unsigned int k = 0; k < bytes_per_pixel_; s_itr[k] += bytes_per_pixel_, ++k)
                {
                    *(itr1[k]) = *(s_itr[k]); itr1[k] += bytes_per_pixel_;
                    *(itr1[k]) = *(s_itr[k]); itr1[k] += bytes_per_pixel_;

                    *(itr2[k]) = *(s_itr[k]); itr2[k] += bytes_per_pixel_;
                    *(itr2[k]) = *(s_itr[k]); itr2[k] += bytes_per_pixel_;
                }
            }

            for (unsigned int k = 0; k < bytes_per_pixel_; ++k)
            {
                itr1[k] += dest.row_increment_;
                itr2[k] += dest.row_increment_;
            }
        }
    }

    inline void alpha_blend(const double& alpha, const bitmap_image& image)
    {
        if (
            (image.width_ != width_) ||
            (image.height_ != height_)
            )
        {
            return;
        }

        if ((alpha < 0.0) || (alpha > 1.0))
        {
            return;
        }

        unsigned char* itr1 = data();
        const unsigned char* itr1_end = end();
        const unsigned char* itr2 = image.data();

        double alpha_compliment = 1.0 - alpha;

        while (itr1 != itr1_end)
        {
            *(itr1) = static_cast<unsigned char>((alpha * (*itr2)) + (alpha_compliment * (*itr1)));
            ++itr1;
            ++itr2;
        }
    }

    inline double psnr(const bitmap_image& image)
    {
        if (
            (image.width_ != width_) ||
            (image.height_ != height_)
            )
        {
            return 0.0;
        }

        const unsigned char* itr1 = data();
        const unsigned char* itr2 = image.data();

        double mse = 0.0;

        while (itr1 != end())
        {
            const double v = (static_cast<double>(*itr1) - static_cast<double>(*itr2));

            mse += v * v;
            ++itr1;
            ++itr2;
        }

        if (mse <= 0.0000001)
        {
            return 1000000.0;
        }
        else
        {
            mse /= (3.0 * width_ * height_);

            return 20.0 * std::log10(255.0 / std::sqrt(mse));
        }
    }

    inline double psnr(const unsigned int& x,
        const unsigned int& y,
        const bitmap_image& image)
    {
        if ((x + image.width()) > width_) { return 0.0; }
        if ((y + image.height()) > height_) { return 0.0; }

        double mse = 0.0;

        const unsigned int height = image.height();
        const unsigned int width = image.width();

        for (unsigned int r = 0; r < height; ++r)
        {
            const unsigned char* itr1 = row(r + y) + x * bytes_per_pixel_;
            const unsigned char* itr1_end = itr1 + (width * bytes_per_pixel_);
            const unsigned char* itr2 = image.row(r);

            while (itr1 != itr1_end)
            {
                double v = (static_cast<double>(*itr1) - static_cast<double>(*itr2));
                mse += v * v;
                ++itr1;
                ++itr2;
            }
        }

        if (mse <= 0.0000001)
        {
            return 1000000.0;
        }
        else
        {
            mse /= (3.0 * image.width() * image.height());
            return 20.0 * std::log10(255.0 / std::sqrt(mse));
        }
    }

    inline void histogram(const color_plane color, double hist[256]) const
    {
        std::fill(hist, hist + 256, 0.0);

        for (const unsigned char* itr = (data() + offset(color)); itr < end(); itr += bytes_per_pixel_)
        {
            ++hist[(*itr)];
        }
    }

    inline void histogram_normalized(const color_plane color, double hist[256]) const
    {
        histogram(color, hist);

        double* h_itr = hist;
        const double* h_end = hist + 256;
        const double pixel_count = static_cast<double>(width_ * height_);

        while (h_end != h_itr)
        {
            *(h_itr++) /= pixel_count;
        }
    }

    inline unsigned int offset(const color_plane color) const
    {
        switch (channel_mode_)
        {
        case rgb_mode: {
            switch (color)
            {
            case red_plane: return 0;
            case green_plane: return 1;
            case blue_plane: return 2;
            default: return std::numeric_limits<unsigned int>::max();
            }
        }

        case bgr_mode: {
            switch (color)
            {
            case red_plane: return 2;
            case green_plane: return 1;
            case blue_plane: return 0;
            default: return std::numeric_limits<unsigned int>::max();
            }
        }

        default: return std::numeric_limits<unsigned int>::max();
        }
    }

    inline void incremental()
    {
        unsigned char current_color = 0;

        for (unsigned char* itr = data(); itr < end();)
        {
            (*itr++) = (current_color);
            (*itr++) = (current_color);
            (*itr++) = (current_color);

            ++current_color;
        }
    }

    inline void reverse_channels()
    {
        if (3 != bytes_per_pixel_)
            return;

        for (unsigned char* itr = data(); itr < end(); itr += bytes_per_pixel_)
        {
            std::swap(*(itr + 0), *(itr + 2));
        }
    }

private:

    inline const unsigned char* end() const
    {
        return data_.data() + data_.size();
    }

    inline unsigned char* end()
    {
        return const_cast<unsigned char*>(data() + data_.size());
    }

    struct bitmap_file_header
    {
        unsigned short type;
        unsigned int   size;
        unsigned short reserved1;
        unsigned short reserved2;
        unsigned int   off_bits;

        unsigned int struct_size() const
        {
            return sizeof(type) +
                sizeof(size) +
                sizeof(reserved1) +
                sizeof(reserved2) +
                sizeof(off_bits);
        }

        void clear()
        {
            std::memset(this, 0x00, sizeof(bitmap_file_header));
        }
    };

    struct bitmap_information_header
    {
        unsigned int   size;
        unsigned int   width;
        unsigned int   height;
        unsigned short planes;
        unsigned short bit_count;
        unsigned int   compression;
        unsigned int   size_image;
        unsigned int   x_pels_per_meter;
        unsigned int   y_pels_per_meter;
        unsigned int   clr_used;
        unsigned int   clr_important;

        unsigned int struct_size() const
        {
            return sizeof(size) +
                sizeof(width) +
                sizeof(height) +
                sizeof(planes) +
                sizeof(bit_count) +
                sizeof(compression) +
                sizeof(size_image) +
                sizeof(x_pels_per_meter) +
                sizeof(y_pels_per_meter) +
                sizeof(clr_used) +
                sizeof(clr_important);
        }

        void clear()
        {
            std::memset(this, 0x00, sizeof(bitmap_information_header));
        }
    };

    inline bool big_endian() const
    {
        unsigned int v = 0x01;

        return (1 != reinterpret_cast<char*>(&v)[0]);
    }

    inline unsigned short flip(const unsigned short& v) const
    {
        return ((v >> 8) | (v << 8));
    }

    inline unsigned int flip(const unsigned int& v) const
    {
        return (
            ((v & 0xFF000000) >> 0x18) |
            ((v & 0x000000FF) << 0x18) |
            ((v & 0x00FF0000) >> 0x08) |
            ((v & 0x0000FF00) << 0x08)
            );
    }

    template <typename T>
    inline void read_from_stream(std::ifstream& stream, T& t)
    {
        stream.read(reinterpret_cast<char*>(&t), sizeof(T));
    }

    template <typename T>
    inline void write_to_stream(std::ofstream& stream, const T& t) const
    {
        stream.write(reinterpret_cast<const char*>(&t), sizeof(T));
    }

    inline void read_bfh(std::ifstream& stream, bitmap_file_header& bfh)
    {
        read_from_stream(stream, bfh.type);
        read_from_stream(stream, bfh.size);
        read_from_stream(stream, bfh.reserved1);
        read_from_stream(stream, bfh.reserved2);
        read_from_stream(stream, bfh.off_bits);

        if (big_endian())
        {
            bfh.type = flip(bfh.type);
            bfh.size = flip(bfh.size);
            bfh.reserved1 = flip(bfh.reserved1);
            bfh.reserved2 = flip(bfh.reserved2);
            bfh.off_bits = flip(bfh.off_bits);
        }
    }

    inline void write_bfh(std::ofstream& stream, const bitmap_file_header& bfh) const
    {
        if (big_endian())
        {
            write_to_stream(stream, flip(bfh.type));
            write_to_stream(stream, flip(bfh.size));
            write_to_stream(stream, flip(bfh.reserved1));
            write_to_stream(stream, flip(bfh.reserved2));
            write_to_stream(stream, flip(bfh.off_bits));
        }
        else
        {
            write_to_stream(stream, bfh.type);
            write_to_stream(stream, bfh.size);
            write_to_stream(stream, bfh.reserved1);
            write_to_stream(stream, bfh.reserved2);
            write_to_stream(stream, bfh.off_bits);
        }
    }

    inline void read_bih(std::ifstream& stream, bitmap_information_header& bih)
    {
        read_from_stream(stream, bih.size);
        read_from_stream(stream, bih.width);
        read_from_stream(stream, bih.height);
        read_from_stream(stream, bih.planes);
        read_from_stream(stream, bih.bit_count);
        read_from_stream(stream, bih.compression);
        read_from_stream(stream, bih.size_image);
        read_from_stream(stream, bih.x_pels_per_meter);
        read_from_stream(stream, bih.y_pels_per_meter);
        read_from_stream(stream, bih.clr_used);
        read_from_stream(stream, bih.clr_important);

        if (big_endian())
        {
            bih.size = flip(bih.size);
            bih.width = flip(bih.width);
            bih.height = flip(bih.height);
            bih.planes = flip(bih.planes);
            bih.bit_count = flip(bih.bit_count);
            bih.compression = flip(bih.compression);
            bih.size_image = flip(bih.size_image);
            bih.x_pels_per_meter = flip(bih.x_pels_per_meter);
            bih.y_pels_per_meter = flip(bih.y_pels_per_meter);
            bih.clr_used = flip(bih.clr_used);
            bih.clr_important = flip(bih.clr_important);
        }
    }

    inline void write_bih(std::ofstream& stream, const bitmap_information_header& bih) const
    {
        if (big_endian())
        {
            write_to_stream(stream, flip(bih.size));
            write_to_stream(stream, flip(bih.width));
            write_to_stream(stream, flip(bih.height));
            write_to_stream(stream, flip(bih.planes));
            write_to_stream(stream, flip(bih.bit_count));
            write_to_stream(stream, flip(bih.compression));
            write_to_stream(stream, flip(bih.size_image));
            write_to_stream(stream, flip(bih.x_pels_per_meter));
            write_to_stream(stream, flip(bih.y_pels_per_meter));
            write_to_stream(stream, flip(bih.clr_used));
            write_to_stream(stream, flip(bih.clr_important));
        }
        else
        {
            write_to_stream(stream, bih.size);
            write_to_stream(stream, bih.width);
            write_to_stream(stream, bih.height);
            write_to_stream(stream, bih.planes);
            write_to_stream(stream, bih.bit_count);
            write_to_stream(stream, bih.compression);
            write_to_stream(stream, bih.size_image);
            write_to_stream(stream, bih.x_pels_per_meter);
            write_to_stream(stream, bih.y_pels_per_meter);
            write_to_stream(stream, bih.clr_used);
            write_to_stream(stream, bih.clr_important);
        }
    }

    inline std::size_t file_size(const std::string& file_name) const
    {
        std::ifstream file(file_name.c_str(), std::ios::in | std::ios::binary);
        if (!file) return 0;
        file.seekg(0, std::ios::end);
        return static_cast<std::size_t>(file.tellg());
    }

    void create_bitmap()
    {
        row_increment_ = width_ * bytes_per_pixel_;
        data_.resize(height_ * row_increment_);
    }

    void load_bitmap()
    {
        std::ifstream stream(file_name_.c_str(), std::ios::binary);

        if (!stream)
        {
            std::cerr << "bitmap_image::load_bitmap() ERROR: bitmap_image - file " << file_name_ << " not found!" << std::endl;
            return;
        }

        width_ = 0;
        height_ = 0;

        bitmap_file_header bfh;
        bitmap_information_header bih;

        bfh.clear();
        bih.clear();

        read_bfh(stream, bfh);
        read_bih(stream, bih);

        if (bfh.type != 19778)
        {
            bfh.clear();
            bih.clear();

            stream.close();

            std::cerr << "bitmap_image::load_bitmap() ERROR: bitmap_image - Invalid type value " << bfh.type << " expected 19778." << std::endl;
            return;
        }

        if (bih.bit_count != 24)
        {
            bfh.clear();
            bih.clear();

            stream.close();

            std::cerr << "bitmap_image::load_bitmap() ERROR: bitmap_image - Invalid bit depth " << bih.bit_count << " expected 24." << std::endl;

            return;
        }

        if (bih.size != bih.struct_size())
        {
            bfh.clear();
            bih.clear();

            stream.close();

            std::cerr << "bitmap_image::load_bitmap() ERROR: bitmap_image - Invalid BIH size " << bih.size << " expected " << bih.struct_size() << std::endl;

            return;
        }

        width_ = bih.width;
        height_ = bih.height;

        bytes_per_pixel_ = bih.bit_count >> 3;

        unsigned int padding = (4 - ((3 * width_) % 4)) % 4;
        char padding_data[4] = { 0,0,0,0 };

        std::size_t bitmap_file_size = file_size(file_name_);

        std::size_t bitmap_logical_size = (height_ * width_ * bytes_per_pixel_) +
            (height_ * padding) +
            bih.struct_size() +
            bfh.struct_size();

        if (bitmap_file_size != bitmap_logical_size)
        {
            bfh.clear();
            bih.clear();

            stream.close();

            std::cerr << "bitmap_image::load_bitmap() ERROR: bitmap_image - Mismatch between logical and physical sizes of bitmap. " <<
                "Logical: " << bitmap_logical_size << " " <<
                "Physical: " << bitmap_file_size << std::endl;

            return;
        }

        create_bitmap();

        for (unsigned int i = 0; i < height_; ++i)
        {
            unsigned char* data_ptr = row(height_ - i - 1); // read in inverted row order

            stream.read(reinterpret_cast<char*>(data_ptr), sizeof(char) * bytes_per_pixel_ * width_);
            stream.read(padding_data, padding);
        }
    }

    template <typename T>
    inline T clamp(const T& v, const T& lower_range, const T& upper_range) const
    {
        if (v < lower_range)
            return lower_range;
        else if (v >  upper_range)
            return upper_range;
        else
            return v;
    }

    std::string  file_name_;
    unsigned int width_;
    unsigned int height_;
    unsigned int row_increment_;
    unsigned int bytes_per_pixel_;
    channel_mode channel_mode_;
    std::vector<unsigned char> data_;
};

typedef bitmap_image::rgb_t rgb_t;

inline bool operator==(const rgb_t& c0, const rgb_t& c1)
{
    return (c0.red == c1.red) &&
        (c0.green == c1.green) &&
        (c0.blue == c1.blue);
}

inline bool operator!=(const rgb_t& c0, const rgb_t& c1)
{
    return (c0.red != c1.red) ||
        (c0.green != c1.green) ||
        (c0.blue != c1.blue);
}

inline std::size_t hamming_distance(const rgb_t& c0, const rgb_t& c1)
{
    std::size_t result = 0;

    if (c0.red != c1.red) ++result;
    if (c0.green != c1.green) ++result;
    if (c0.blue != c1.blue) ++result;

    return result;
}

inline rgb_t make_colour(const unsigned int& red, const unsigned int& green, const unsigned int& blue)
{
    rgb_t result;

    result.red = static_cast<unsigned char>(red);
    result.green = static_cast<unsigned char>(green);
    result.blue = static_cast<unsigned char>(blue);

    return result;
}

template <typename OutputIterator>
inline void generate_colours(const std::size_t& steps, const rgb_t c0, const rgb_t& c1, OutputIterator out)
{
    double dr = ((double)c1.red - (double)c0.red) / steps;
    double dg = ((double)c1.green - (double)c0.green) / steps;
    double db = ((double)c1.blue - (double)c0.blue) / steps;

    for (std::size_t i = 0; i < steps; ++i)
    {
        rgb_t c;

        c.red = static_cast<unsigned char>(c0.red + (i * dr));
        c.green = static_cast<unsigned char>(c0.green + (i * dg));
        c.blue = static_cast<unsigned char>(c0.blue + (i * db));

        *(out++) = c;
    }
}

template <typename ResponseImage, typename Palette>
inline std::size_t convert_rsp_to_image(const ResponseImage& resp_image, const Palette& palette, bitmap_image& image)
{
    if (
        (resp_image.width() > image.width()) ||
        (resp_image.height() > image.height())
        )
        return 0;

    for (std::size_t y = 0; y < resp_image.height(); ++y)
    {
        for (std::size_t x = 0; x < resp_image.width(); ++x)
        {
            const double v = resp_image(x, y);

            unsigned int index = static_cast<unsigned int>((v < 0) ? 0 : v >(palette.size()) ? (palette.size() - 1) : v);

            image.set_pixel(x, y, palette[index]);
        }
    }

    return (resp_image.width() * resp_image.height());
}

inline void rgb_to_ycbcr(const unsigned int& length, double* red, double* green, double* blue,
    double* y, double* cb, double* cr)
{
    unsigned int i = 0;

    while (i < length)
    {
        (*y) = 16.0 + (65.481 * (*red) + 128.553 * (*green) + 24.966 * (*blue));
        (*cb) = 128.0 + (-37.797 * (*red) + -74.203 * (*green) + 112.000 * (*blue));
        (*cr) = 128.0 + (112.000 * (*red) + -93.786 * (*green) - 18.214 * (*blue));

        ++i;
        ++red; ++green; ++blue;
        ++y;   ++cb;    ++cr;
    }
}

inline void ycbcr_to_rgb(const unsigned int& length, double* y, double* cb, double* cr,
    double* red, double* green, double* blue)
{
    unsigned int i = 0;

    while (i < length)
    {
        double y_ = (*y) - 16.0;
        double cb_ = (*cb) - 128.0;
        double cr_ = (*cr) - 128.0;

        (*red) = 0.000456621 * y_ + 0.00625893 * cr_;
        (*green) = 0.000456621 * y_ - 0.00153632 * cb_ - 0.00318811 * cr_;
        (*blue) = 0.000456621 * y_ + 0.00791071 * cb_;

        ++i;
        ++red; ++green; ++blue;
        ++y;   ++cb;    ++cr;
    }
}

inline void subsample(const unsigned int& width,
    const unsigned int& height,
    const double* source,
    unsigned int& w,
    unsigned int& h,
    double*& dest)
{
    /*  Single channel.  */

    w = 0;
    h = 0;

    bool odd_width = false;
    bool odd_height = false;

    if (0 == (width % 2))
        w = width / 2;
    else
    {
        w = 1 + (width / 2);
        odd_width = true;
    }

    if (0 == (height % 2))
        h = height / 2;
    else
    {
        h = 1 + (height / 2);
        odd_height = true;
    }

    unsigned int horizontal_upper = (odd_width) ? w - 1 : w;
    unsigned int vertical_upper = (odd_height) ? h - 1 : h;

    dest = new double[w * h];

    double* s_itr = dest;
    const double* itr1 = source;
    const double* itr2 = source + width;

    for (unsigned int j = 0; j < vertical_upper; ++j)
    {
        for (unsigned int i = 0; i < horizontal_upper; ++i, ++s_itr)
        {
            (*s_itr) = *(itr1++);
            (*s_itr) += *(itr1++);
            (*s_itr) += *(itr2++);
            (*s_itr) += *(itr2++);
            (*s_itr) /= 4.0;
        }

        if (odd_width)
        {
            (*(s_itr++)) = ((*itr1++) + (*itr2++)) / 2.0;
        }

        itr1 += width;

        if (j != (vertical_upper - 1))
        {
            itr2 += width;
        }
    }

    if (odd_height)
    {
        for (unsigned int i = 0; i < horizontal_upper; ++i, ++s_itr)
        {
            (*s_itr) += (*(itr1++));
            (*s_itr) += (*(itr1++));
            (*s_itr) /= 2.0;
        }

        if (odd_width)
        {
            (*(s_itr++)) = (*itr1);
        }
    }
}

inline void upsample(const unsigned int& width,
    const unsigned int& height,
    const double* source,
    unsigned int& w,
    unsigned int& h,
    double*& dest)
{
    /* Single channel. */

    w = 2 * width;
    h = 2 * height;

    dest = new double[w * h];

    const double* s_itr = source;
    double* itr1 = dest;
    double* itr2 = dest + w;

    for (unsigned int j = 0; j < height; ++j)
    {
        for (unsigned int i = 0; i < width; ++i, ++s_itr)
        {
            *(itr1++) = (*s_itr);
            *(itr1++) = (*s_itr);
            *(itr2++) = (*s_itr);
            *(itr2++) = (*s_itr);
        }

        itr1 += w;
        itr2 += w;
    }
}

inline void checkered_pattern(const unsigned int x_width,
    const unsigned int y_width,
    const unsigned char value,
    const bitmap_image::color_plane color,
    bitmap_image& image)
{
    if (
        (x_width >= image.width()) ||
        (y_width >= image.height())
        )
    {
        return;
    }

    bool setter_x = false;
    bool setter_y = true;

    const unsigned int color_plane_offset = image.offset(color);
    const unsigned int height = image.height();
    const unsigned int width = image.width();

    for (unsigned int y = 0; y < height; ++y)
    {
        if (0 == (y % y_width))
        {
            setter_y = !setter_y;
        }

        unsigned char* row = image.row(y) + color_plane_offset;

        for (unsigned int x = 0; x < width; ++x, row += image.bytes_per_pixel())
        {
            if (0 == (x % x_width))
            {
                setter_x = !setter_x;
            }

            if (setter_x ^ setter_y)
            {
                *row = value;
            }
        }
    }
}

inline void checkered_pattern(const unsigned int x_width,
    const unsigned int y_width,
    const unsigned char red,
    const unsigned char green,
    const unsigned char blue,
    bitmap_image& image)
{
    if (
        (x_width >= image.width()) ||
        (y_width >= image.height())
        )
    {
        return;
    }

    bool setter_x = false;
    bool setter_y = true;

    const unsigned int height = image.height();
    const unsigned int width = image.width();

    for (unsigned int y = 0; y < height; ++y)
    {
        if (0 == (y % y_width))
        {
            setter_y = !setter_y;
        }

        unsigned char* row = image.row(y);

        for (unsigned int x = 0; x < width; ++x, row += image.bytes_per_pixel())
        {
            if (0 == (x % x_width))
            {
                setter_x = !setter_x;
            }

            if (setter_x ^ setter_y)
            {
                *(row + 0) = blue;
                *(row + 1) = green;
                *(row + 2) = red;
            }
        }
    }
}

inline void plasma(bitmap_image& image,
    const double& x, const double& y,
    const double& width, const double& height,
    const double& c1, const double& c2,
    const double& c3, const double& c4,
    const double& roughness = 3.0,
    const rgb_t   colormap[] = 0)
{
    // Note: c1,c2,c3,c4 -> [0.0,1.0]

    const double half_width = (width / 2.0);
    const double half_height = (height / 2.0);

    if ((width >= 1.0) || (height >= 1.0))
    {
        const double corner1 = (c1 + c2) / 2.0;
        const double corner2 = (c2 + c3) / 2.0;
        const double corner3 = (c3 + c4) / 2.0;
        const double corner4 = (c4 + c1) / 2.0;
        double center = (c1 + c2 + c3 + c4) / 4.0 +
            ((1.0 * ::rand() / (1.0 * RAND_MAX)) - 0.5) * // should use a better rng
            ((1.0 * half_width + half_height) / (image.width() + image.height()) * roughness);

        center = std::min<double>(std::max<double>(0.0, center), 1.0);

        plasma(image, x, y, half_width, half_height, c1, corner1, center, corner4, roughness, colormap);
        plasma(image, x + half_width, y, half_width, half_height, corner1, c2, corner2, center, roughness, colormap);
        plasma(image, x + half_width, y + half_height, half_width, half_height, center, corner2, c3, corner3, roughness, colormap);
        plasma(image, x, y + half_height, half_width, half_height, corner4, center, corner3, c4, roughness, colormap);
    }
    else
    {
        rgb_t color = colormap[static_cast<unsigned int>(1000.0 * ((c1 + c2 + c3 + c4) / 4.0)) % 1000];

        image.set_pixel(static_cast<unsigned int>(x), static_cast<unsigned int>(y), color);
    }
}

inline void plasma(bitmap_image& image,
    const double& c1, const double& c2,
    const double& c3, const double& c4,
    const double& roughness = 3.0,
    const rgb_t   colormap[] = 0)
{
    plasma
    (
        image, 0, 0, image.width(), image.height(),
        c1, c2, c3, c4,
        roughness, colormap
    );
}

inline double psnr_region(const unsigned int& x, const unsigned int& y,
    const unsigned int& width, const unsigned int& height,
    const bitmap_image& image1, const bitmap_image& image2)
{
    if (
        (image1.width() != image2.width()) ||
        (image1.height() != image2.height())
        )
    {
        return 0.0;
    }

    if ((x + width) > image1.width()) { return 0.0; }
    if ((y + height) > image1.height()) { return 0.0; }

    double mse = 0.0;

    for (unsigned int r = 0; r < height; ++r)
    {
        const unsigned char* itr1 = image1.row(r + y) + x * image1.bytes_per_pixel();
        const unsigned char* itr1_end = itr1 + (width * image1.bytes_per_pixel());
        const unsigned char* itr2 = image2.row(r + y) + x * image2.bytes_per_pixel();

        while (itr1 != itr1_end)
        {
            double v = (static_cast<double>(*itr1) - static_cast<double>(*itr2));
            mse += v * v;
            ++itr1;
            ++itr2;
        }
    }

    if (mse <= 0.0000001)
    {
        return 1000000.0;
    }
    else
    {
        mse /= (3.0 * width * height);
        return 20.0 * std::log10(255.0 / std::sqrt(mse));
    }
}

inline void hierarchical_psnr_r(const double& x, const double& y,
    const double& width, const double& height,
    const bitmap_image& image1,
    bitmap_image& image2,
    const double& threshold,
    const rgb_t colormap[])
{
    if ((width <= 4.0) || (height <= 4.0))
    {
        const double psnr = psnr_region
        (
            static_cast<unsigned int>(x),
            static_cast<unsigned int>(y),
            static_cast<unsigned int>(width),
            static_cast<unsigned int>(height),
            image1, image2
        );

        if (psnr < threshold)
        {
            rgb_t c = colormap[static_cast<unsigned int>(1000.0 * (1.0 - (psnr / threshold)))];

            image2.set_region
            (
                static_cast<unsigned int>(x),
                static_cast<unsigned int>(y),
                static_cast<unsigned int>(width + 1),
                static_cast<unsigned int>(height + 1),
                c.red, c.green, c.blue
            );
        }
    }
    else
    {
        const double half_width = (width / 2.0);
        const double half_height = (height / 2.0);

        hierarchical_psnr_r(x, y, half_width, half_height, image1, image2, threshold, colormap);
        hierarchical_psnr_r(x + half_width, y, half_width, half_height, image1, image2, threshold, colormap);
        hierarchical_psnr_r(x + half_width, y + half_height, half_width, half_height, image1, image2, threshold, colormap);
        hierarchical_psnr_r(x, y + half_height, half_width, half_height, image1, image2, threshold, colormap);
    }
}

inline void hierarchical_psnr(bitmap_image& image1, bitmap_image& image2, const double threshold, const rgb_t colormap[])
{
    if (
        (image1.width() != image2.width()) ||
        (image1.height() != image2.height())
        )
    {
        return;
    }

    const double psnr = psnr_region
    (
        0, 0, image1.width(), image1.height(),
        image1, image2
    );

    if (psnr < threshold)
    {
        hierarchical_psnr_r
        (
            0, 0, image1.width(), image1.height(),
            image1, image2,
            threshold,
            colormap
        );
    }
}

class image_drawer
{
public:

    image_drawer(bitmap_image& image)
        : image_(image),
        pen_width_(1),
        pen_color_red_(0),
        pen_color_green_(0),
        pen_color_blue_(0)
    {}

    void rectangle(int x1, int y1, int x2, int y2)
    {
        line_segment(x1, y1, x2, y1);
        line_segment(x2, y1, x2, y2);
        line_segment(x2, y2, x1, y2);
        line_segment(x1, y2, x1, y1);
    }

    void triangle(int x1, int y1, int x2, int y2, int x3, int y3)
    {
        line_segment(x1, y1, x2, y2);
        line_segment(x2, y2, x3, y3);
        line_segment(x3, y3, x1, y1);
    }

    void quadix(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4)
    {
        line_segment(x1, y1, x2, y2);
        line_segment(x2, y2, x3, y3);
        line_segment(x3, y3, x4, y4);
        line_segment(x4, y4, x1, y1);
    }

    void line_segment(int x1, int y1, int x2, int y2)
    {
        int steep = 0;
        int sx = ((x2 - x1) > 0) ? 1 : -1;
        int sy = ((y2 - y1) > 0) ? 1 : -1;
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);

        if (dy > dx)
        {
            std::swap(x1, y1);
            std::swap(dx, dy);
            std::swap(sx, sy);

            steep = 1;
        }

        int e = 2 * dy - dx;

        for (int i = 0; i < dx; ++i)
        {
            if (steep)
                plot_pen_pixel(y1, x1);
            else
                plot_pen_pixel(x1, y1);

            while (e >= 0)
            {
                y1 += sy;
                e -= (dx << 1);
            }

            x1 += sx;
            e += (dy << 1);
        }

        plot_pen_pixel(x2, y2);
    }

    void horiztonal_line_segment(int x1, int x2, int y)
    {
        if (x1 > x2)
        {
            std::swap(x1, x2);
        }

        for (int i = 0; i < (x2 - x1); ++i)
        {
            plot_pen_pixel(x1 + i, y);
        }
    }

    void vertical_line_segment(int y1, int y2, int x)
    {
        if (y1 > y2)
        {
            std::swap(y1, y2);
        }

        for (int i = 0; i < (y2 - y1); ++i)
        {
            plot_pen_pixel(x, y1 + i);
        }
    }

    void ellipse(int centerx, int centery, int a, int b)
    {
        int t1 = a * a;
        int t2 = t1 << 1;
        int t3 = t2 << 1;
        int t4 = b * b;
        int t5 = t4 << 1;
        int t6 = t5 << 1;
        int t7 = a * t5;
        int t8 = t7 << 1;
        int t9 = 0;

        int d1 = t2 - t7 + (t4 >> 1);
        int d2 = (t1 >> 1) - t8 + t5;
        int x = a;
        int y = 0;

        int negative_tx = centerx - x;
        int positive_tx = centerx + x;
        int negative_ty = centery - y;
        int positive_ty = centery + y;

        while (d2 < 0)
        {
            plot_pen_pixel(positive_tx, positive_ty);
            plot_pen_pixel(positive_tx, negative_ty);
            plot_pen_pixel(negative_tx, positive_ty);
            plot_pen_pixel(negative_tx, negative_ty);

            ++y;

            t9 = t9 + t3;

            if (d1 < 0)
            {
                d1 = d1 + t9 + t2;
                d2 = d2 + t9;
            }
            else
            {
                x--;
                t8 = t8 - t6;
                d1 = d1 + (t9 + t2 - t8);
                d2 = d2 + (t9 + t5 - t8);
                negative_tx = centerx - x;
                positive_tx = centerx + x;
            }

            negative_ty = centery - y;
            positive_ty = centery + y;
        }

        do
        {
            plot_pen_pixel(positive_tx, positive_ty);
            plot_pen_pixel(positive_tx, negative_ty);
            plot_pen_pixel(negative_tx, positive_ty);
            plot_pen_pixel(negative_tx, negative_ty);

            x--;
            t8 = t8 - t6;

            if (d2 < 0)
            {
                ++y;
                t9 = t9 + t3;
                d2 = d2 + (t9 + t5 - t8);
                negative_ty = centery - y;
                positive_ty = centery + y;
            }
            else
                d2 = d2 + (t5 - t8);

            negative_tx = centerx - x;
            positive_tx = centerx + x;
        } while (x >= 0);
    }

    void circle(int centerx, int centery, int radius)
    {
        int x = 0;
        int d = (1 - radius) << 1;

        while (radius >= 0)
        {
            plot_pen_pixel(centerx + x, centery + radius);
            plot_pen_pixel(centerx + x, centery - radius);
            plot_pen_pixel(centerx - x, centery + radius);
            plot_pen_pixel(centerx - x, centery - radius);

            if ((d + radius) > 0)
                d -= ((--radius) << 1) - 1;
            if (x > d)
                d += ((++x) << 1) + 1;
        }
    }

    void plot_pen_pixel(int x, int y)
    {
        switch (pen_width_)
        {
        case 1: plot_pixel(x, y);
            break;

        case 2: {
            plot_pixel(x, y);
            plot_pixel(x + 1, y);
            plot_pixel(x + 1, y + 1);
            plot_pixel(x, y + 1);
        }
                break;

        case  3: {
            plot_pixel(x, y - 1);
            plot_pixel(x - 1, y - 1);
            plot_pixel(x + 1, y - 1);

            plot_pixel(x, y);
            plot_pixel(x - 1, y);
            plot_pixel(x + 1, y);

            plot_pixel(x, y + 1);
            plot_pixel(x - 1, y + 1);
            plot_pixel(x + 1, y + 1);
        }
                 break;

        default: plot_pixel(x, y);
            break;
        }
    }

    void plot_pixel(int x, int y)
    {
        if (
            (x < 0) ||
            (y < 0) ||
            (x >= static_cast<int>(image_.width())) ||
            (y >= static_cast<int>(image_.height()))
            )
            return;

        image_.set_pixel(x, y, pen_color_red_, pen_color_green_, pen_color_blue_);
    }

    void pen_width(const unsigned int& width)
    {
        if ((width > 0) && (width < 4))
        {
            pen_width_ = width;
        }
    }

    void pen_color(const unsigned char& red,
        const unsigned char& green,
        const unsigned char& blue)
    {
        pen_color_red_ = red;
        pen_color_green_ = green;
        pen_color_blue_ = blue;
    }

    template <typename RGB>
    void pen_color(const RGB colour)
    {
        pen_color_red_ = colour.red;
        pen_color_green_ = colour.green;
        pen_color_blue_ = colour.blue;
    }

private:

    image_drawer(const image_drawer& id);
    image_drawer& operator =(const image_drawer& id);

    bitmap_image& image_;
    unsigned int  pen_width_;
    unsigned char pen_color_red_;
    unsigned char pen_color_green_;
    unsigned char pen_color_blue_;
};

class cartesian_canvas
{
public:

    cartesian_canvas(const double x_length, const double y_length)
        : width_div2_(0.0),
        height_div2_(0.0),
        min_x_(0.0),
        min_y_(0.0),
        max_x_(0.0),
        max_y_(0.0),
        draw_(image_)
    {
        setup_canvas(x_length, y_length);
    }

    inline bool operator!()
    {
        return !image_;
    }

    void rectangle(double x1, double y1, double x2, double y2)
    {
        line_segment(x1, y1, x2, y1);
        line_segment(x2, y1, x2, y2);
        line_segment(x2, y2, x1, y2);
        line_segment(x1, y2, x1, y1);
    }

    void triangle(double x1, double y1, double x2, double y2, double x3, double y3)
    {
        line_segment(x1, y1, x2, y2);
        line_segment(x2, y2, x3, y3);
        line_segment(x3, y3, x1, y1);
    }

    void quadix(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
    {
        line_segment(x1, y1, x2, y2);
        line_segment(x2, y2, x3, y3);
        line_segment(x3, y3, x4, y4);
        line_segment(x4, y4, x1, y1);
    }

    void line_segment(double x1, double y1, double x2, double y2)
    {
        if (clip(x1, y1, x2, y2))
        {
            const int sc_x1 = static_cast<int>(cart_to_screen_x(x1));
            const int sc_x2 = static_cast<int>(cart_to_screen_x(x2));
            const int sc_y1 = static_cast<int>(cart_to_screen_y(y1));
            const int sc_y2 = static_cast<int>(cart_to_screen_y(y2));

            draw_.line_segment(sc_x1, sc_y1, sc_x2, sc_y2);
        }
    }

    void horiztonal_line_segment(double x1, double x2, double y)
    {
        x1 = clamp_x(x1);
        x2 = clamp_x(x2);
        y = clamp_y(y);

        const int sc_x1 = static_cast<int>(cart_to_screen_x(x1));
        const int sc_x2 = static_cast<int>(cart_to_screen_x(x2));
        const int sc_y = static_cast<int>(cart_to_screen_y(y));

        draw_.horiztonal_line_segment(sc_x1, sc_x2, sc_y);
    }

    void vertical_line_segment(double y1, double y2, double x)
    {
        y1 = clamp_y(y1);
        y2 = clamp_y(y2);
        x = clamp_x(x);

        const int sc_y1 = static_cast<int>(cart_to_screen_y(y1));
        const int sc_y2 = static_cast<int>(cart_to_screen_y(y2));
        const int sc_x = static_cast<int>(cart_to_screen_x(x));

        draw_.vertical_line_segment(sc_y1, sc_y2, sc_x);
    }

    void ellipse(double centerx, double centery, double a, double b)
    {

        const int sc_cx = static_cast<int>(cart_to_screen_x(centerx));
        const int sc_cy = static_cast<int>(cart_to_screen_y(centery));

        draw_.ellipse(sc_cx, sc_cy, static_cast<int>(a), static_cast<int>(b));
    }

    void circle(double centerx, double centery, double radius)
    {
        const int sc_cx = static_cast<int>(cart_to_screen_x(centerx));
        const int sc_cy = static_cast<int>(cart_to_screen_y(centery));

        draw_.circle(sc_cx, sc_cy, static_cast<int>(radius));
    }

    void fill_rectangle(double x1, double y1, double x2, double y2)
    {
        if (y1 > y2)
            std::swap(y1, y2);

        for (double y = y1; y <= y2; y += 0.5)
        {
            line_segment(x1, y, x2, y);
        }
    }

    void fill_triangle(double x1, double y1, double x2, double y2, double x3, double y3)
    {
        typedef std::pair<double, double> point_t;

        std::vector<point_t> p;

        p.push_back(std::make_pair(x1, y1));
        p.push_back(std::make_pair(x2, y2));
        p.push_back(std::make_pair(x3, y3));

        if (p[0].second > p[1].second)
            std::swap(p[0], p[1]);
        if (p[0].second > p[2].second)
            std::swap(p[0], p[2]);
        if (p[1].second > p[2].second)
            std::swap(p[1], p[2]);

        class draw_modes
        {
        private:

            cartesian_canvas & canvas;

            // Needed for incompetent and broken msvc compiler versions
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4822)
#endif
            draw_modes& operator=(const draw_modes&);
#ifdef _MSC_VER
#pragma warning(pop)
#endif

        public:

            draw_modes(cartesian_canvas& c)
                : canvas(c)
            {}

            void bottom(const point_t& p0, const point_t& p1, const point_t& p2)
            {
                const double m0 = (p1.first - p0.first) / (2.0 * (p1.second - p0.second));
                const double m1 = (p2.first - p0.first) / (2.0 * (p2.second - p0.second));

                double x0 = p0.first;
                double x1 = p0.first;

                for (double y = p0.second; y <= p1.second; y += 0.5)
                {
                    canvas.horiztonal_line_segment(x0, x1, y);

                    x0 += m0;
                    x1 += m1;
                }
            }

            void top(const point_t& p0, const point_t& p1, const point_t& p2)
            {
                const double m0 = (p2.first - p0.first) / (2.0 * (p2.second - p0.second));
                const double m1 = (p2.first - p1.first) / (2.0 * (p2.second - p1.second));

                double x0 = p2.first;
                double x1 = p2.first;

                for (double y = p2.second; y >= p0.second; y -= 0.5)
                {
                    canvas.horiztonal_line_segment(x0, x1, y);

                    x0 -= m0;
                    x1 -= m1;
                }
            }
        };

        draw_modes dm(*this);

        const double eps = 0.00001;

        if (std::abs(p[1].second - p[2].second) < eps)
            dm.bottom(p[0], p[1], p[2]);
        else if (std::abs(p[0].second - p[1].second) < eps)
            dm.top(p[0], p[1], p[2]);
        else
        {
            point_t p3;

            p3.first = (p[0].first + ((p[1].second - p[0].second) / (p[2].second - p[0].second)) * (p[2].first - p[0].first));
            p3.second = p[1].second;

            dm.bottom(p[0], p[1], p3);
            dm.top(p[1], p3, p[2]);
        }
    }

    void fill_quadix(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
    {
        fill_triangle(x1, y1, x2, y2, x3, y3);
        fill_triangle(x1, y1, x3, y3, x4, y4);
    }

    void fill_circle(double cx, double cy, double radius)
    {
        const double delta = 1.0;
        double  x = radius;
        double  y = 0.0;
        double dx = delta - (2.0 * delta * radius);
        double dy = 0.0;
        double dr = 0.0;

        while (x >= y)
        {
            for (double i = cx - x; i <= cx + x; i += delta)
            {
                horiztonal_line_segment(cx - x, cx + x, cy + y);
                horiztonal_line_segment(cx - x, cx + x, cy - y);
            }

            for (double i = cx - y; i <= cx + y; i += delta)
            {
                horiztonal_line_segment(cx - y, cx + y, cy + x);
                horiztonal_line_segment(cx - y, cx + y, cy - x);
            }

            y += delta;

            dr += dy;
            dy += 2.0 * delta;

            if ((2.0 * delta * dr + dx) > 0)
            {
                x -= delta;
                dr += dx;
                dx += 2.0 * delta;
            }
        }
    }

    void plot_pen_pixel(double x, double y)
    {
        if ((x < min_x_) || (x > max_x_)) return;
        if ((y < min_y_) || (y > max_y_)) return;

        const int sc_x = static_cast<int>(cart_to_screen_x(x));
        const int sc_y = static_cast<int>(cart_to_screen_y(y));

        draw_.plot_pen_pixel(sc_x, sc_y);
    }

    void plot_pixel(double x, double y)
    {
        if ((x < min_x_) || (x > max_x_)) return;
        if ((y < min_y_) || (y > max_y_)) return;

        const int sc_x = static_cast<int>(cart_to_screen_x(x));
        const int sc_y = static_cast<int>(cart_to_screen_y(y));

        draw_.plot_pixel(sc_x, sc_y);
    }

    void pen_width(const unsigned int& width)
    {
        draw_.pen_width(width);
    }

    void pen_color(const unsigned char&   red,
        const unsigned char& green,
        const unsigned char&  blue)
    {
        draw_.pen_color(red, green, blue);
    }

    template <typename RGB>
    void pen_color(const RGB colour)
    {
        draw_.pen_color(colour);
    }

    const bitmap_image& image() const
    {
        return image_;
    }

    bitmap_image& image()
    {
        return image_;
    }

    void set_widthheight(const double x_length, const double y_length)
    {
        setup_canvas(x_length, y_length);
    }

    double min_x() const { return min_x_; }
    double min_y() const { return min_y_; }
    double max_x() const { return max_x_; }
    double max_y() const { return max_y_; }

private:

    void setup_canvas(const double x_length, const double y_length)
    {
        if ((x_length < 2.0) || (y_length < 2.0))
            return;

        width_div2_ = x_length / 2.0;
        height_div2_ = y_length / 2.0;

        min_x_ = -width_div2_;
        min_y_ = -height_div2_;
        max_x_ = width_div2_;
        max_y_ = height_div2_;

        image_.setwidth_height(static_cast<unsigned int>(x_length) + 1, static_cast<unsigned int>(y_length) + 1);

        image_.clear(0xFF);
    }

    double clamp_x(const double& x)
    {
        if (x < min_x_)  return min_x_;
        else if (x > max_x_)  return max_x_;
        else                  return x;
    }

    double clamp_y(const double& y)
    {
        if (y < min_y_)  return min_y_;
        else if (y > max_y_)  return max_y_;
        else                  return y;
    }

    double cart_to_screen_x(const double& x)
    {
        return x + width_div2_;
    }

    double cart_to_screen_y(const double& y)
    {
        return height_div2_ - y;
    }

    enum clip_code
    {
        e_clip_bottom = 1,
        e_clip_top = 2,
        e_clip_left = 4,
        e_clip_right = 8
    };

    int out_code(
        const double&  x, const double&  y,
        const double& x1, const double& y1,
        const double& x2, const double& y2
    )
    {
        int result = 0;
        if (y < y1)      result |= e_clip_bottom;
        else if (y > y2) result |= e_clip_top;

        if (x < x1)      result |= e_clip_left;
        else if (x > x2) result |= e_clip_right;

        return result;
    }

    bool clip(double& x1, double& y1, double& x2, double& y2)
    {
        bool   result = false;
        double x = 0.0;
        double y = 0.0;

        int outcode0 = out_code(x1, y1, min_x_, min_y_, max_x_, max_y_);
        int outcode1 = out_code(x2, y2, min_x_, min_y_, max_x_, max_y_);
        int outcodeout = 0;

        while ((outcode0 != 0) || (outcode1 != 0))
        {
            if ((outcode0 & outcode1) != 0)
                return result;
            else
            {
                if (outcode0 != 0)
                    outcodeout = outcode0;
                else
                    outcodeout = outcode1;

                double dx = (x2 - x1);
                double dy = (y2 - y1);

                if ((outcodeout & e_clip_bottom) == e_clip_bottom)
                {
                    x = x1 + dx * (min_y_ - y1) / dy;
                    y = min_y_;
                }
                else if ((outcodeout & e_clip_top) == e_clip_top)
                {
                    x = x1 + dx * (max_y_ - y1) / dy;
                    y = max_y_;
                }
                else if ((outcodeout & e_clip_right) == e_clip_right)
                {
                    y = y1 + dy * (max_x_ - x1) / dx;
                    x = max_x_;
                }
                else if ((outcodeout & e_clip_left) == e_clip_left)
                {
                    y = y1 + dy * (min_x_ - x1) / dx;
                    x = min_x_;
                }

                if (outcodeout == outcode0)
                {
                    x1 = x;
                    y1 = y;
                    outcode0 = out_code(x1, y1, min_x_, min_y_, max_x_, max_y_);
                }
                else
                {
                    x2 = x;
                    y2 = y;
                    outcode1 = out_code(x2, y2, min_x_, min_y_, max_x_, max_y_);
                }
            }
        }

        return true;
    }

    cartesian_canvas(const cartesian_canvas&);
    cartesian_canvas operator=(const cartesian_canvas&);

    double width_div2_;
    double height_div2_;
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
    bitmap_image image_;
    image_drawer draw_;
};

inline rgb_t convert_wave_length_nm_to_rgb(const double wave_length_nm)
{
    // Credits: Dan Bruton http://www.physics.sfasu.edu/astro/color.html
    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;

    if ((380.0 <= wave_length_nm) && (wave_length_nm <= 439.0))
    {
        red = -(wave_length_nm - 440.0) / (440.0 - 380.0);
        green = 0.0;
        blue = 1.0;
    }
    else if ((440.0 <= wave_length_nm) && (wave_length_nm <= 489.0))
    {
        red = 0.0;
        green = (wave_length_nm - 440.0) / (490.0 - 440.0);
        blue = 1.0;
    }
    else if ((490.0 <= wave_length_nm) && (wave_length_nm <= 509.0))
    {
        red = 0.0;
        green = 1.0;
        blue = -(wave_length_nm - 510.0) / (510.0 - 490.0);
    }
    else if ((510.0 <= wave_length_nm) && (wave_length_nm <= 579.0))
    {
        red = (wave_length_nm - 510.0) / (580.0 - 510.0);
        green = 1.0;
        blue = 0.0;
    }
    else if ((580.0 <= wave_length_nm) && (wave_length_nm <= 644.0))
    {
        red = 1.0;
        green = -(wave_length_nm - 645.0) / (645.0 - 580.0);
        blue = 0.0;
    }
    else if ((645.0 <= wave_length_nm) && (wave_length_nm <= 780.0))
    {
        red = 1.0;
        green = 0.0;
        blue = 0.0;
    }

    double factor = 0.0;

    if ((380.0 <= wave_length_nm) && (wave_length_nm <= 419.0))
        factor = 0.3 + 0.7 * (wave_length_nm - 380.0) / (420.0 - 380.0);
    else if ((420.0 <= wave_length_nm) && (wave_length_nm <= 700.0))
        factor = 1.0;
    else if ((701.0 <= wave_length_nm) && (wave_length_nm <= 780.0))
        factor = 0.3 + 0.7 * (780.0 - wave_length_nm) / (780.0 - 700.0);
    else
        factor = 0.0;

    rgb_t result;

    const double gamma = 0.8;
    const double intensity_max = 255.0;

#define round(d) std::floor(d + 0.5)

    result.red = static_cast<unsigned char>((red == 0.0) ? red : round(intensity_max * std::pow(red   * factor, gamma)));
    result.green = static_cast<unsigned char>((green == 0.0) ? green : round(intensity_max * std::pow(green * factor, gamma)));
    result.blue = static_cast<unsigned char>((blue == 0.0) ? blue : round(intensity_max * std::pow(blue  * factor, gamma)));

#undef round

    return result;
}

inline double weighted_distance(const unsigned char r0, const unsigned char g0, const unsigned char b0,
    const unsigned char r1, const unsigned char g1, const unsigned char b1)
{
    const double diff_r = /*0.30 */ (r0 - r1);
    const double diff_g = /*0.59 */ (g0 - g1);
    const double diff_b = /*0.11 */ (b0 - b1);

    return std::sqrt((diff_r * diff_r) + (diff_g * diff_g) + (diff_b * diff_b));
}

inline double weighted_distance(const rgb_t c0, const rgb_t c1)
{
    return weighted_distance(c0.red, c0.green, c0.blue,
        c1.red, c1.green, c1.blue);
}

template <typename Iterator>
inline rgb_t find_nearest_color(const rgb_t& c, const Iterator begin, const Iterator end)
{
    if (0 == std::distance(begin, end))
        return c;

    double min_d = std::numeric_limits<double>::max();
    rgb_t result = *begin;

    for (Iterator itr = begin; itr != end; ++itr)
    {
        if (c == (*itr))
        {
            return (*itr);
        }

        double curr_d = weighted_distance(c, *itr);

        if (curr_d < min_d)
        {
            min_d = curr_d;
            result = *itr;
        }
    }

    return result;
}

template <template <typename, typename> class Sequence,
    typename Allocator>
    inline rgb_t find_nearest_color(const rgb_t& c, const Sequence<rgb_t, Allocator>& seq)
{
    return find_nearest_color(c, seq.begin(), seq.end());
}

template <std::size_t N>
inline rgb_t find_nearest_color(const rgb_t& c, const rgb_t(&colors)[N])
{
    return find_nearest_color(c, colors, colors + N);
}

inline double find_nearest_wave_length(const rgb_t& c, const double increment = 0.001)
{
    const double max_wave_length = 800.0; //800nm

    double min_wave_length = 0.0;
    double min_d = std::numeric_limits<double>::max();

    for (double i = 0.0; i < max_wave_length; i += increment)
    {
        const rgb_t  curr_rgb = convert_wave_length_nm_to_rgb(i);

        if (c == curr_rgb)
        {
            return i;
        }

        const double curr_d = weighted_distance(c, curr_rgb);

        if (curr_d <= min_d)
        {
            min_wave_length = i;
            min_d = curr_d;
        }
    }

    return min_wave_length;
}

template <typename T>
class response_image
{
public:

    response_image(const std::size_t& width, const std::size_t& height, const T null = T(0))
        : width_(width),
        height_(height),
        null_(null)
    {
        data_.resize(width_ * height_);
    }

    std::size_t width() const { return  width_; }
    std::size_t height() const { return height_; }

    void set_all(const T& t)
    {
        std::fill_n(data_.begin(), data_.size(), t);
    }

    const T& operator()(const std::size_t& x, const std::size_t& y) const
    {
        if (y >= height_) return null_;
        if (x >= width_) return null_;

        return data_[width_ * y + x];
    }

    T& operator()(const std::size_t& x, const std::size_t& y)
    {
        if (y >= height_) return null_;
        if (x >= width_) return null_;

        return data_[width_ * y + x];
    }

    bool valid(const std::size_t& x, const std::size_t& y)
    {
        return ((x < width_) || (y < height_));
    }

    void inc_all(const T& v)
    {
        for (std::size_t i = 0; i < data_.size(); ++i)
        {
            data_[i] += v;
        }
    }

    void mul_all(const T& v)
    {
        for (std::size_t i = 0; i < data_.size(); ++i)
        {
            data_[i] *= v;
        }
    }

    T* row(const std::size_t& row_index)
    {
        if (row_index < height_)
            return &data_[width_ * row_index];
        else
            return reinterpret_cast<T*>(0);
    }

    const T* row(const std::size_t& row_index) const
    {
        if (row_index < height_)
            return data_[width_ * row_index];
        else
            return reinterpret_cast<T*>(0);
    }

private:

    std::size_t    width_;
    std::size_t    height_;
    std::vector<T> data_;
    T              null_;
};

inline void sobel_operator(const bitmap_image& src_image,
    bitmap_image& dst_image,
    const double threshold = 0.0)
{
    typedef double T;

    response_image<T> im0(src_image.width(), src_image.height(), 0.0);
    response_image<T> im1(src_image.width(), src_image.height(), 0.0);

    src_image.export_gray_scale_response_image(&im0(0, 0));

    for (std::size_t y = 1; y < im0.height() - 1; ++y)
    {
        const T* itr0 = im0.row(y - 1);
        const T* itr1 = im0.row(y);
        const T* itr2 = im0.row(y + 1);
        T* out = im1.row(y) + 1;

        for (std::size_t x = 1; x < im0.width() - 1; ++x)
        {
            const T c0 = *(itr0 + x - 1);   const T c1 = *(itr0 + x);   const T c2 = *(itr0 + x + 1);
            const T c3 = *(itr1 + x - 1); /*const T c4 = *(itr1 + x);*/ const T c5 = *(itr1 + x + 1);
            const T c6 = *(itr2 + x - 1);   const T c7 = *(itr2 + x);   const T c8 = *(itr2 + x + 1);

            const T gx = (2.0 * (c5 - c3)) + (c2 - c0) + (c8 - c6);
            const T gy = (2.0 * (c1 - c7)) + (c0 - c6) + (c2 - c8);

            *(out++) = std::sqrt((gx * gx) + (gy * gy));
        }
    }

    if (threshold > 0.0)
    {
        const T* end = im1.row(0) + (im1.width() * im1.height());

        for (T* itr = im1.row(0); itr != end; ++itr)
        {
            T& v = *itr;
            if (v <= threshold) v = 0;
        }
    }

    dst_image.setwidth_height
    (
        static_cast<unsigned int>(im1.width()),
        static_cast<unsigned int>(im1.height())
    );

    dst_image.import_gray_scale_clamped(&im1(0, 0));
}

enum palette_name
{
    e_red, e_scarlet, e_vermilion, e_tangelo, e_orange,
    e_gamboge, e_amber, e_gold, e_yellow, e_apple_green,
    e_lime_green, e_spring_bud, e_chartreuse_green, e_pistachio, e_harlequin,
    e_sap_green, e_green, e_emerald_green, e_malachite_green, e_sea_green,
    e_spring_green, e_aquamarine, e_turquoise, e_opal, e_cyan,
    e_arctic_blue, e_cerulean, e_cornflower_blue, e_azure, e_cobalt_blue,
    e_sapphire_blue, e_phthalo_blue, e_blue, e_persian_blue, e_indigo,
    e_blue_violet, e_violet, e_purple, e_mulberry, e_heliotrope,
    e_magenta, e_orchid, e_fuchsia, e_cerise, e_rose,
    e_raspberry, e_crimson, e_amaranth, e_white, e_black
};

const rgb_t palette_colormap[] = {
    { 255,   0,   0 },{ 255,  31,   0 },{ 255,  63,   0 },{ 255,  95,   0 },{ 255, 127,   0 },
{ 255, 159,   0 },{ 255, 191,   0 },{ 255, 223,   0 },{ 255, 255,   0 },{ 223, 255,   0 },
{ 191, 255,   0 },{ 159, 255,   0 },{ 127, 255,   0 },{ 95, 255,   0 },{ 63, 255,   0 },
{ 31, 255,   0 },{ 0, 255,   0 },{ 0, 255,  31 },{ 0, 255,  63 },{ 0, 255,  95 },
{ 0, 255, 127 },{ 0, 255, 159 },{ 0, 255, 191 },{ 0, 255, 223 },{ 0, 255, 255 },
{ 0, 223, 255 },{ 0, 191, 255 },{ 0, 159, 255 },{ 0, 127, 255 },{ 0,  95, 255 },
{ 0,  63, 255 },{ 0,  31, 255 },{ 0,   0, 255 },{ 31,   0, 255 },{ 63,   0, 255 },
{ 95,   0, 255 },{ 127,   0, 255 },{ 159,   0, 255 },{ 191,   0, 255 },{ 223,   0, 255 },
{ 255,   0, 255 },{ 255,   0, 223 },{ 255,   0, 191 },{ 255,   0, 159 },{ 255,   0, 127 },
{ 255,   0,  95 },{ 255,   0,  63 },{ 255,   0,  31 },{ 255, 255, 255 },{ 0,   0,   0 }
};

const rgb_t autumn_colormap[1000] = {
    { 255,   0,   0 },{ 255,   0,   0 },{ 255,   1,   0 },{ 255,   1,   0 },{ 255,   1,   0 },
{ 255,   1,   0 },{ 255,   2,   0 },{ 255,   2,   0 },{ 255,   2,   0 },{ 255,   2,   0 },
{ 255,   3,   0 },{ 255,   3,   0 },{ 255,   3,   0 },{ 255,   3,   0 },{ 255,   4,   0 },
{ 255,   4,   0 },{ 255,   4,   0 },{ 255,   4,   0 },{ 255,   5,   0 },{ 255,   5,   0 },
{ 255,   5,   0 },{ 255,   5,   0 },{ 255,   6,   0 },{ 255,   6,   0 },{ 255,   6,   0 },
{ 255,   6,   0 },{ 255,   7,   0 },{ 255,   7,   0 },{ 255,   7,   0 },{ 255,   7,   0 },
{ 255,   8,   0 },{ 255,   8,   0 },{ 255,   8,   0 },{ 255,   8,   0 },{ 255,   9,   0 },
{ 255,   9,   0 },{ 255,   9,   0 },{ 255,   9,   0 },{ 255,  10,   0 },{ 255,  10,   0 },
{ 255,  10,   0 },{ 255,  10,   0 },{ 255,  11,   0 },{ 255,  11,   0 },{ 255,  11,   0 },
{ 255,  11,   0 },{ 255,  12,   0 },{ 255,  12,   0 },{ 255,  12,   0 },{ 255,  13,   0 },
{ 255,  13,   0 },{ 255,  13,   0 },{ 255,  13,   0 },{ 255,  14,   0 },{ 255,  14,   0 },
{ 255,  14,   0 },{ 255,  14,   0 },{ 255,  15,   0 },{ 255,  15,   0 },{ 255,  15,   0 },
{ 255,  15,   0 },{ 255,  16,   0 },{ 255,  16,   0 },{ 255,  16,   0 },{ 255,  16,   0 },
{ 255,  17,   0 },{ 255,  17,   0 },{ 255,  17,   0 },{ 255,  17,   0 },{ 255,  18,   0 },
{ 255,  18,   0 },{ 255,  18,   0 },{ 255,  18,   0 },{ 255,  19,   0 },{ 255,  19,   0 },
{ 255,  19,   0 },{ 255,  19,   0 },{ 255,  20,   0 },{ 255,  20,   0 },{ 255,  20,   0 },
{ 255,  20,   0 },{ 255,  21,   0 },{ 255,  21,   0 },{ 255,  21,   0 },{ 255,  21,   0 },
{ 255,  22,   0 },{ 255,  22,   0 },{ 255,  22,   0 },{ 255,  22,   0 },{ 255,  23,   0 },
{ 255,  23,   0 },{ 255,  23,   0 },{ 255,  23,   0 },{ 255,  24,   0 },{ 255,  24,   0 },
{ 255,  24,   0 },{ 255,  25,   0 },{ 255,  25,   0 },{ 255,  25,   0 },{ 255,  25,   0 },
{ 255,  26,   0 },{ 255,  26,   0 },{ 255,  26,   0 },{ 255,  26,   0 },{ 255,  27,   0 },
{ 255,  27,   0 },{ 255,  27,   0 },{ 255,  27,   0 },{ 255,  28,   0 },{ 255,  28,   0 },
{ 255,  28,   0 },{ 255,  28,   0 },{ 255,  29,   0 },{ 255,  29,   0 },{ 255,  29,   0 },
{ 255,  29,   0 },{ 255,  30,   0 },{ 255,  30,   0 },{ 255,  30,   0 },{ 255,  30,   0 },
{ 255,  31,   0 },{ 255,  31,   0 },{ 255,  31,   0 },{ 255,  31,   0 },{ 255,  32,   0 },
{ 255,  32,   0 },{ 255,  32,   0 },{ 255,  32,   0 },{ 255,  33,   0 },{ 255,  33,   0 },
{ 255,  33,   0 },{ 255,  33,   0 },{ 255,  34,   0 },{ 255,  34,   0 },{ 255,  34,   0 },
{ 255,  34,   0 },{ 255,  35,   0 },{ 255,  35,   0 },{ 255,  35,   0 },{ 255,  35,   0 },
{ 255,  36,   0 },{ 255,  36,   0 },{ 255,  36,   0 },{ 255,  37,   0 },{ 255,  37,   0 },
{ 255,  37,   0 },{ 255,  37,   0 },{ 255,  38,   0 },{ 255,  38,   0 },{ 255,  38,   0 },
{ 255,  38,   0 },{ 255,  39,   0 },{ 255,  39,   0 },{ 255,  39,   0 },{ 255,  39,   0 },
{ 255,  40,   0 },{ 255,  40,   0 },{ 255,  40,   0 },{ 255,  40,   0 },{ 255,  41,   0 },
{ 255,  41,   0 },{ 255,  41,   0 },{ 255,  41,   0 },{ 255,  42,   0 },{ 255,  42,   0 },
{ 255,  42,   0 },{ 255,  42,   0 },{ 255,  43,   0 },{ 255,  43,   0 },{ 255,  43,   0 },
{ 255,  43,   0 },{ 255,  44,   0 },{ 255,  44,   0 },{ 255,  44,   0 },{ 255,  44,   0 },
{ 255,  45,   0 },{ 255,  45,   0 },{ 255,  45,   0 },{ 255,  45,   0 },{ 255,  46,   0 },
{ 255,  46,   0 },{ 255,  46,   0 },{ 255,  46,   0 },{ 255,  47,   0 },{ 255,  47,   0 },
{ 255,  47,   0 },{ 255,  47,   0 },{ 255,  48,   0 },{ 255,  48,   0 },{ 255,  48,   0 },
{ 255,  48,   0 },{ 255,  49,   0 },{ 255,  49,   0 },{ 255,  49,   0 },{ 255,  50,   0 },
{ 255,  50,   0 },{ 255,  50,   0 },{ 255,  50,   0 },{ 255,  51,   0 },{ 255,  51,   0 },
{ 255,  51,   0 },{ 255,  51,   0 },{ 255,  52,   0 },{ 255,  52,   0 },{ 255,  52,   0 },
{ 255,  52,   0 },{ 255,  53,   0 },{ 255,  53,   0 },{ 255,  53,   0 },{ 255,  53,   0 },
{ 255,  54,   0 },{ 255,  54,   0 },{ 255,  54,   0 },{ 255,  54,   0 },{ 255,  55,   0 },
{ 255,  55,   0 },{ 255,  55,   0 },{ 255,  55,   0 },{ 255,  56,   0 },{ 255,  56,   0 },
{ 255,  56,   0 },{ 255,  56,   0 },{ 255,  57,   0 },{ 255,  57,   0 },{ 255,  57,   0 },
{ 255,  57,   0 },{ 255,  58,   0 },{ 255,  58,   0 },{ 255,  58,   0 },{ 255,  58,   0 },
{ 255,  59,   0 },{ 255,  59,   0 },{ 255,  59,   0 },{ 255,  59,   0 },{ 255,  60,   0 },
{ 255,  60,   0 },{ 255,  60,   0 },{ 255,  60,   0 },{ 255,  61,   0 },{ 255,  61,   0 },
{ 255,  61,   0 },{ 255,  62,   0 },{ 255,  62,   0 },{ 255,  62,   0 },{ 255,  62,   0 },
{ 255,  63,   0 },{ 255,  63,   0 },{ 255,  63,   0 },{ 255,  63,   0 },{ 255,  64,   0 },
{ 255,  64,   0 },{ 255,  64,   0 },{ 255,  64,   0 },{ 255,  65,   0 },{ 255,  65,   0 },
{ 255,  65,   0 },{ 255,  65,   0 },{ 255,  66,   0 },{ 255,  66,   0 },{ 255,  66,   0 },
{ 255,  66,   0 },{ 255,  67,   0 },{ 255,  67,   0 },{ 255,  67,   0 },{ 255,  67,   0 },
{ 255,  68,   0 },{ 255,  68,   0 },{ 255,  68,   0 },{ 255,  68,   0 },{ 255,  69,   0 },
{ 255,  69,   0 },{ 255,  69,   0 },{ 255,  69,   0 },{ 255,  70,   0 },{ 255,  70,   0 },
{ 255,  70,   0 },{ 255,  70,   0 },{ 255,  71,   0 },{ 255,  71,   0 },{ 255,  71,   0 },
{ 255,  71,   0 },{ 255,  72,   0 },{ 255,  72,   0 },{ 255,  72,   0 },{ 255,  72,   0 },
{ 255,  73,   0 },{ 255,  73,   0 },{ 255,  73,   0 },{ 255,  74,   0 },{ 255,  74,   0 },
{ 255,  74,   0 },{ 255,  74,   0 },{ 255,  75,   0 },{ 255,  75,   0 },{ 255,  75,   0 },
{ 255,  75,   0 },{ 255,  76,   0 },{ 255,  76,   0 },{ 255,  76,   0 },{ 255,  76,   0 },
{ 255,  77,   0 },{ 255,  77,   0 },{ 255,  77,   0 },{ 255,  77,   0 },{ 255,  78,   0 },
{ 255,  78,   0 },{ 255,  78,   0 },{ 255,  78,   0 },{ 255,  79,   0 },{ 255,  79,   0 },
{ 255,  79,   0 },{ 255,  79,   0 },{ 255,  80,   0 },{ 255,  80,   0 },{ 255,  80,   0 },
{ 255,  80,   0 },{ 255,  81,   0 },{ 255,  81,   0 },{ 255,  81,   0 },{ 255,  81,   0 },
{ 255,  82,   0 },{ 255,  82,   0 },{ 255,  82,   0 },{ 255,  82,   0 },{ 255,  83,   0 },
{ 255,  83,   0 },{ 255,  83,   0 },{ 255,  83,   0 },{ 255,  84,   0 },{ 255,  84,   0 },
{ 255,  84,   0 },{ 255,  84,   0 },{ 255,  85,   0 },{ 255,  85,   0 },{ 255,  85,   0 },
{ 255,  86,   0 },{ 255,  86,   0 },{ 255,  86,   0 },{ 255,  86,   0 },{ 255,  87,   0 },
{ 255,  87,   0 },{ 255,  87,   0 },{ 255,  87,   0 },{ 255,  88,   0 },{ 255,  88,   0 },
{ 255,  88,   0 },{ 255,  88,   0 },{ 255,  89,   0 },{ 255,  89,   0 },{ 255,  89,   0 },
{ 255,  89,   0 },{ 255,  90,   0 },{ 255,  90,   0 },{ 255,  90,   0 },{ 255,  90,   0 },
{ 255,  91,   0 },{ 255,  91,   0 },{ 255,  91,   0 },{ 255,  91,   0 },{ 255,  92,   0 },
{ 255,  92,   0 },{ 255,  92,   0 },{ 255,  92,   0 },{ 255,  93,   0 },{ 255,  93,   0 },
{ 255,  93,   0 },{ 255,  93,   0 },{ 255,  94,   0 },{ 255,  94,   0 },{ 255,  94,   0 },
{ 255,  94,   0 },{ 255,  95,   0 },{ 255,  95,   0 },{ 255,  95,   0 },{ 255,  95,   0 },
{ 255,  96,   0 },{ 255,  96,   0 },{ 255,  96,   0 },{ 255,  96,   0 },{ 255,  97,   0 },
{ 255,  97,   0 },{ 255,  97,   0 },{ 255,  98,   0 },{ 255,  98,   0 },{ 255,  98,   0 },
{ 255,  98,   0 },{ 255,  99,   0 },{ 255,  99,   0 },{ 255,  99,   0 },{ 255,  99,   0 },
{ 255, 100,   0 },{ 255, 100,   0 },{ 255, 100,   0 },{ 255, 100,   0 },{ 255, 101,   0 },
{ 255, 101,   0 },{ 255, 101,   0 },{ 255, 101,   0 },{ 255, 102,   0 },{ 255, 102,   0 },
{ 255, 102,   0 },{ 255, 102,   0 },{ 255, 103,   0 },{ 255, 103,   0 },{ 255, 103,   0 },
{ 255, 103,   0 },{ 255, 104,   0 },{ 255, 104,   0 },{ 255, 104,   0 },{ 255, 104,   0 },
{ 255, 105,   0 },{ 255, 105,   0 },{ 255, 105,   0 },{ 255, 105,   0 },{ 255, 106,   0 },
{ 255, 106,   0 },{ 255, 106,   0 },{ 255, 106,   0 },{ 255, 107,   0 },{ 255, 107,   0 },
{ 255, 107,   0 },{ 255, 107,   0 },{ 255, 108,   0 },{ 255, 108,   0 },{ 255, 108,   0 },
{ 255, 108,   0 },{ 255, 109,   0 },{ 255, 109,   0 },{ 255, 109,   0 },{ 255, 110,   0 },
{ 255, 110,   0 },{ 255, 110,   0 },{ 255, 110,   0 },{ 255, 111,   0 },{ 255, 111,   0 },
{ 255, 111,   0 },{ 255, 111,   0 },{ 255, 112,   0 },{ 255, 112,   0 },{ 255, 112,   0 },
{ 255, 112,   0 },{ 255, 113,   0 },{ 255, 113,   0 },{ 255, 113,   0 },{ 255, 113,   0 },
{ 255, 114,   0 },{ 255, 114,   0 },{ 255, 114,   0 },{ 255, 114,   0 },{ 255, 115,   0 },
{ 255, 115,   0 },{ 255, 115,   0 },{ 255, 115,   0 },{ 255, 116,   0 },{ 255, 116,   0 },
{ 255, 116,   0 },{ 255, 116,   0 },{ 255, 117,   0 },{ 255, 117,   0 },{ 255, 117,   0 },
{ 255, 117,   0 },{ 255, 118,   0 },{ 255, 118,   0 },{ 255, 118,   0 },{ 255, 118,   0 },
{ 255, 119,   0 },{ 255, 119,   0 },{ 255, 119,   0 },{ 255, 119,   0 },{ 255, 120,   0 },
{ 255, 120,   0 },{ 255, 120,   0 },{ 255, 120,   0 },{ 255, 121,   0 },{ 255, 121,   0 },
{ 255, 121,   0 },{ 255, 122,   0 },{ 255, 122,   0 },{ 255, 122,   0 },{ 255, 122,   0 },
{ 255, 123,   0 },{ 255, 123,   0 },{ 255, 123,   0 },{ 255, 123,   0 },{ 255, 124,   0 },
{ 255, 124,   0 },{ 255, 124,   0 },{ 255, 124,   0 },{ 255, 125,   0 },{ 255, 125,   0 },
{ 255, 125,   0 },{ 255, 125,   0 },{ 255, 126,   0 },{ 255, 126,   0 },{ 255, 126,   0 },
{ 255, 126,   0 },{ 255, 127,   0 },{ 255, 127,   0 },{ 255, 127,   0 },{ 255, 127,   0 },
{ 255, 128,   0 },{ 255, 128,   0 },{ 255, 128,   0 },{ 255, 128,   0 },{ 255, 129,   0 },
{ 255, 129,   0 },{ 255, 129,   0 },{ 255, 129,   0 },{ 255, 130,   0 },{ 255, 130,   0 },
{ 255, 130,   0 },{ 255, 130,   0 },{ 255, 131,   0 },{ 255, 131,   0 },{ 255, 131,   0 },
{ 255, 131,   0 },{ 255, 132,   0 },{ 255, 132,   0 },{ 255, 132,   0 },{ 255, 132,   0 },
{ 255, 133,   0 },{ 255, 133,   0 },{ 255, 133,   0 },{ 255, 133,   0 },{ 255, 134,   0 },
{ 255, 134,   0 },{ 255, 134,   0 },{ 255, 135,   0 },{ 255, 135,   0 },{ 255, 135,   0 },
{ 255, 135,   0 },{ 255, 136,   0 },{ 255, 136,   0 },{ 255, 136,   0 },{ 255, 136,   0 },
{ 255, 137,   0 },{ 255, 137,   0 },{ 255, 137,   0 },{ 255, 137,   0 },{ 255, 138,   0 },
{ 255, 138,   0 },{ 255, 138,   0 },{ 255, 138,   0 },{ 255, 139,   0 },{ 255, 139,   0 },
{ 255, 139,   0 },{ 255, 139,   0 },{ 255, 140,   0 },{ 255, 140,   0 },{ 255, 140,   0 },
{ 255, 140,   0 },{ 255, 141,   0 },{ 255, 141,   0 },{ 255, 141,   0 },{ 255, 141,   0 },
{ 255, 142,   0 },{ 255, 142,   0 },{ 255, 142,   0 },{ 255, 142,   0 },{ 255, 143,   0 },
{ 255, 143,   0 },{ 255, 143,   0 },{ 255, 143,   0 },{ 255, 144,   0 },{ 255, 144,   0 },
{ 255, 144,   0 },{ 255, 144,   0 },{ 255, 145,   0 },{ 255, 145,   0 },{ 255, 145,   0 },
{ 255, 145,   0 },{ 255, 146,   0 },{ 255, 146,   0 },{ 255, 146,   0 },{ 255, 147,   0 },
{ 255, 147,   0 },{ 255, 147,   0 },{ 255, 147,   0 },{ 255, 148,   0 },{ 255, 148,   0 },
{ 255, 148,   0 },{ 255, 148,   0 },{ 255, 149,   0 },{ 255, 149,   0 },{ 255, 149,   0 },
{ 255, 149,   0 },{ 255, 150,   0 },{ 255, 150,   0 },{ 255, 150,   0 },{ 255, 150,   0 },
{ 255, 151,   0 },{ 255, 151,   0 },{ 255, 151,   0 },{ 255, 151,   0 },{ 255, 152,   0 },
{ 255, 152,   0 },{ 255, 152,   0 },{ 255, 152,   0 },{ 255, 153,   0 },{ 255, 153,   0 },
{ 255, 153,   0 },{ 255, 153,   0 },{ 255, 154,   0 },{ 255, 154,   0 },{ 255, 154,   0 },
{ 255, 154,   0 },{ 255, 155,   0 },{ 255, 155,   0 },{ 255, 155,   0 },{ 255, 155,   0 },
{ 255, 156,   0 },{ 255, 156,   0 },{ 255, 156,   0 },{ 255, 156,   0 },{ 255, 157,   0 },
{ 255, 157,   0 },{ 255, 157,   0 },{ 255, 157,   0 },{ 255, 158,   0 },{ 255, 158,   0 },
{ 255, 158,   0 },{ 255, 159,   0 },{ 255, 159,   0 },{ 255, 159,   0 },{ 255, 159,   0 },
{ 255, 160,   0 },{ 255, 160,   0 },{ 255, 160,   0 },{ 255, 160,   0 },{ 255, 161,   0 },
{ 255, 161,   0 },{ 255, 161,   0 },{ 255, 161,   0 },{ 255, 162,   0 },{ 255, 162,   0 },
{ 255, 162,   0 },{ 255, 162,   0 },{ 255, 163,   0 },{ 255, 163,   0 },{ 255, 163,   0 },
{ 255, 163,   0 },{ 255, 164,   0 },{ 255, 164,   0 },{ 255, 164,   0 },{ 255, 164,   0 },
{ 255, 165,   0 },{ 255, 165,   0 },{ 255, 165,   0 },{ 255, 165,   0 },{ 255, 166,   0 },
{ 255, 166,   0 },{ 255, 166,   0 },{ 255, 166,   0 },{ 255, 167,   0 },{ 255, 167,   0 },
{ 255, 167,   0 },{ 255, 167,   0 },{ 255, 168,   0 },{ 255, 168,   0 },{ 255, 168,   0 },
{ 255, 168,   0 },{ 255, 169,   0 },{ 255, 169,   0 },{ 255, 169,   0 },{ 255, 169,   0 },
{ 255, 170,   0 },{ 255, 170,   0 },{ 255, 170,   0 },{ 255, 171,   0 },{ 255, 171,   0 },
{ 255, 171,   0 },{ 255, 171,   0 },{ 255, 172,   0 },{ 255, 172,   0 },{ 255, 172,   0 },
{ 255, 172,   0 },{ 255, 173,   0 },{ 255, 173,   0 },{ 255, 173,   0 },{ 255, 173,   0 },
{ 255, 174,   0 },{ 255, 174,   0 },{ 255, 174,   0 },{ 255, 174,   0 },{ 255, 175,   0 },
{ 255, 175,   0 },{ 255, 175,   0 },{ 255, 175,   0 },{ 255, 176,   0 },{ 255, 176,   0 },
{ 255, 176,   0 },{ 255, 176,   0 },{ 255, 177,   0 },{ 255, 177,   0 },{ 255, 177,   0 },
{ 255, 177,   0 },{ 255, 178,   0 },{ 255, 178,   0 },{ 255, 178,   0 },{ 255, 178,   0 },
{ 255, 179,   0 },{ 255, 179,   0 },{ 255, 179,   0 },{ 255, 179,   0 },{ 255, 180,   0 },
{ 255, 180,   0 },{ 255, 180,   0 },{ 255, 180,   0 },{ 255, 181,   0 },{ 255, 181,   0 },
{ 255, 181,   0 },{ 255, 181,   0 },{ 255, 182,   0 },{ 255, 182,   0 },{ 255, 182,   0 },
{ 255, 183,   0 },{ 255, 183,   0 },{ 255, 183,   0 },{ 255, 183,   0 },{ 255, 184,   0 },
{ 255, 184,   0 },{ 255, 184,   0 },{ 255, 184,   0 },{ 255, 185,   0 },{ 255, 185,   0 },
{ 255, 185,   0 },{ 255, 185,   0 },{ 255, 186,   0 },{ 255, 186,   0 },{ 255, 186,   0 },
{ 255, 186,   0 },{ 255, 187,   0 },{ 255, 187,   0 },{ 255, 187,   0 },{ 255, 187,   0 },
{ 255, 188,   0 },{ 255, 188,   0 },{ 255, 188,   0 },{ 255, 188,   0 },{ 255, 189,   0 },
{ 255, 189,   0 },{ 255, 189,   0 },{ 255, 189,   0 },{ 255, 190,   0 },{ 255, 190,   0 },
{ 255, 190,   0 },{ 255, 190,   0 },{ 255, 191,   0 },{ 255, 191,   0 },{ 255, 191,   0 },
{ 255, 191,   0 },{ 255, 192,   0 },{ 255, 192,   0 },{ 255, 192,   0 },{ 255, 192,   0 },
{ 255, 193,   0 },{ 255, 193,   0 },{ 255, 193,   0 },{ 255, 193,   0 },{ 255, 194,   0 },
{ 255, 194,   0 },{ 255, 194,   0 },{ 255, 195,   0 },{ 255, 195,   0 },{ 255, 195,   0 },
{ 255, 195,   0 },{ 255, 196,   0 },{ 255, 196,   0 },{ 255, 196,   0 },{ 255, 196,   0 },
{ 255, 197,   0 },{ 255, 197,   0 },{ 255, 197,   0 },{ 255, 197,   0 },{ 255, 198,   0 },
{ 255, 198,   0 },{ 255, 198,   0 },{ 255, 198,   0 },{ 255, 199,   0 },{ 255, 199,   0 },
{ 255, 199,   0 },{ 255, 199,   0 },{ 255, 200,   0 },{ 255, 200,   0 },{ 255, 200,   0 },
{ 255, 200,   0 },{ 255, 201,   0 },{ 255, 201,   0 },{ 255, 201,   0 },{ 255, 201,   0 },
{ 255, 202,   0 },{ 255, 202,   0 },{ 255, 202,   0 },{ 255, 202,   0 },{ 255, 203,   0 },
{ 255, 203,   0 },{ 255, 203,   0 },{ 255, 203,   0 },{ 255, 204,   0 },{ 255, 204,   0 },
{ 255, 204,   0 },{ 255, 204,   0 },{ 255, 205,   0 },{ 255, 205,   0 },{ 255, 205,   0 },
{ 255, 205,   0 },{ 255, 206,   0 },{ 255, 206,   0 },{ 255, 206,   0 },{ 255, 207,   0 },
{ 255, 207,   0 },{ 255, 207,   0 },{ 255, 207,   0 },{ 255, 208,   0 },{ 255, 208,   0 },
{ 255, 208,   0 },{ 255, 208,   0 },{ 255, 209,   0 },{ 255, 209,   0 },{ 255, 209,   0 },
{ 255, 209,   0 },{ 255, 210,   0 },{ 255, 210,   0 },{ 255, 210,   0 },{ 255, 210,   0 },
{ 255, 211,   0 },{ 255, 211,   0 },{ 255, 211,   0 },{ 255, 211,   0 },{ 255, 212,   0 },
{ 255, 212,   0 },{ 255, 212,   0 },{ 255, 212,   0 },{ 255, 213,   0 },{ 255, 213,   0 },
{ 255, 213,   0 },{ 255, 213,   0 },{ 255, 214,   0 },{ 255, 214,   0 },{ 255, 214,   0 },
{ 255, 214,   0 },{ 255, 215,   0 },{ 255, 215,   0 },{ 255, 215,   0 },{ 255, 215,   0 },
{ 255, 216,   0 },{ 255, 216,   0 },{ 255, 216,   0 },{ 255, 216,   0 },{ 255, 217,   0 },
{ 255, 217,   0 },{ 255, 217,   0 },{ 255, 217,   0 },{ 255, 218,   0 },{ 255, 218,   0 },
{ 255, 218,   0 },{ 255, 218,   0 },{ 255, 219,   0 },{ 255, 219,   0 },{ 255, 219,   0 },
{ 255, 220,   0 },{ 255, 220,   0 },{ 255, 220,   0 },{ 255, 220,   0 },{ 255, 221,   0 },
{ 255, 221,   0 },{ 255, 221,   0 },{ 255, 221,   0 },{ 255, 222,   0 },{ 255, 222,   0 },
{ 255, 222,   0 },{ 255, 222,   0 },{ 255, 223,   0 },{ 255, 223,   0 },{ 255, 223,   0 },
{ 255, 223,   0 },{ 255, 224,   0 },{ 255, 224,   0 },{ 255, 224,   0 },{ 255, 224,   0 },
{ 255, 225,   0 },{ 255, 225,   0 },{ 255, 225,   0 },{ 255, 225,   0 },{ 255, 226,   0 },
{ 255, 226,   0 },{ 255, 226,   0 },{ 255, 226,   0 },{ 255, 227,   0 },{ 255, 227,   0 },
{ 255, 227,   0 },{ 255, 227,   0 },{ 255, 228,   0 },{ 255, 228,   0 },{ 255, 228,   0 },
{ 255, 228,   0 },{ 255, 229,   0 },{ 255, 229,   0 },{ 255, 229,   0 },{ 255, 229,   0 },
{ 255, 230,   0 },{ 255, 230,   0 },{ 255, 230,   0 },{ 255, 230,   0 },{ 255, 231,   0 },
{ 255, 231,   0 },{ 255, 231,   0 },{ 255, 232,   0 },{ 255, 232,   0 },{ 255, 232,   0 },
{ 255, 232,   0 },{ 255, 233,   0 },{ 255, 233,   0 },{ 255, 233,   0 },{ 255, 233,   0 },
{ 255, 234,   0 },{ 255, 234,   0 },{ 255, 234,   0 },{ 255, 234,   0 },{ 255, 235,   0 },
{ 255, 235,   0 },{ 255, 235,   0 },{ 255, 235,   0 },{ 255, 236,   0 },{ 255, 236,   0 },
{ 255, 236,   0 },{ 255, 236,   0 },{ 255, 237,   0 },{ 255, 237,   0 },{ 255, 237,   0 },
{ 255, 237,   0 },{ 255, 238,   0 },{ 255, 238,   0 },{ 255, 238,   0 },{ 255, 238,   0 },
{ 255, 239,   0 },{ 255, 239,   0 },{ 255, 239,   0 },{ 255, 239,   0 },{ 255, 240,   0 },
{ 255, 240,   0 },{ 255, 240,   0 },{ 255, 240,   0 },{ 255, 241,   0 },{ 255, 241,   0 },
{ 255, 241,   0 },{ 255, 241,   0 },{ 255, 242,   0 },{ 255, 242,   0 },{ 255, 242,   0 },
{ 255, 242,   0 },{ 255, 243,   0 },{ 255, 243,   0 },{ 255, 243,   0 },{ 255, 244,   0 },
{ 255, 244,   0 },{ 255, 244,   0 },{ 255, 244,   0 },{ 255, 245,   0 },{ 255, 245,   0 },
{ 255, 245,   0 },{ 255, 245,   0 },{ 255, 246,   0 },{ 255, 246,   0 },{ 255, 246,   0 },
{ 255, 246,   0 },{ 255, 247,   0 },{ 255, 247,   0 },{ 255, 247,   0 },{ 255, 247,   0 },
{ 255, 248,   0 },{ 255, 248,   0 },{ 255, 248,   0 },{ 255, 248,   0 },{ 255, 249,   0 },
{ 255, 249,   0 },{ 255, 249,   0 },{ 255, 249,   0 },{ 255, 250,   0 },{ 255, 250,   0 },
{ 255, 250,   0 },{ 255, 250,   0 },{ 255, 251,   0 },{ 255, 251,   0 },{ 255, 251,   0 },
{ 255, 251,   0 },{ 255, 252,   0 },{ 255, 252,   0 },{ 255, 252,   0 },{ 255, 252,   0 },
{ 255, 253,   0 },{ 255, 253,   0 },{ 255, 253,   0 },{ 255, 253,   0 },{ 255, 254,   0 },
{ 255, 254,   0 },{ 255, 254,   0 },{ 255, 254,   0 },{ 255, 255,   0 },{ 255, 255,   0 }
};

const rgb_t copper_colormap[1000] = {
    { 0,   0,   0 },{ 0,   0,   0 },{ 1,   0,   0 },{ 1,   1,   0 },{ 1,   1,   1 },
{ 2,   1,   1 },{ 2,   1,   1 },{ 2,   1,   1 },{ 3,   2,   1 },{ 3,   2,   1 },
{ 3,   2,   1 },{ 4,   2,   1 },{ 4,   2,   2 },{ 4,   3,   2 },{ 4,   3,   2 },
{ 5,   3,   2 },{ 5,   3,   2 },{ 5,   3,   2 },{ 6,   4,   2 },{ 6,   4,   2 },
{ 6,   4,   3 },{ 7,   4,   3 },{ 7,   4,   3 },{ 7,   5,   3 },{ 8,   5,   3 },
{ 8,   5,   3 },{ 8,   5,   3 },{ 9,   5,   3 },{ 9,   6,   4 },{ 9,   6,   4 },
{ 10,   6,   4 },{ 10,   6,   4 },{ 10,   6,   4 },{ 11,   7,   4 },{ 11,   7,   4 },
{ 11,   7,   4 },{ 11,   7,   5 },{ 12,   7,   5 },{ 12,   8,   5 },{ 12,   8,   5 },
{ 13,   8,   5 },{ 13,   8,   5 },{ 13,   8,   5 },{ 14,   9,   5 },{ 14,   9,   6 },
{ 14,   9,   6 },{ 15,   9,   6 },{ 15,   9,   6 },{ 15,  10,   6 },{ 16,  10,   6 },
{ 16,  10,   6 },{ 16,  10,   6 },{ 17,  10,   7 },{ 17,  11,   7 },{ 17,  11,   7 },
{ 18,  11,   7 },{ 18,  11,   7 },{ 18,  11,   7 },{ 19,  12,   7 },{ 19,  12,   7 },
{ 19,  12,   8 },{ 19,  12,   8 },{ 20,  12,   8 },{ 20,  13,   8 },{ 20,  13,   8 },
{ 21,  13,   8 },{ 21,  13,   8 },{ 21,  13,   9 },{ 22,  14,   9 },{ 22,  14,   9 },
{ 22,  14,   9 },{ 23,  14,   9 },{ 23,  14,   9 },{ 23,  15,   9 },{ 24,  15,   9 },
{ 24,  15,  10 },{ 24,  15,  10 },{ 25,  15,  10 },{ 25,  16,  10 },{ 25,  16,  10 },
{ 26,  16,  10 },{ 26,  16,  10 },{ 26,  16,  10 },{ 26,  17,  11 },{ 27,  17,  11 },
{ 27,  17,  11 },{ 27,  17,  11 },{ 28,  17,  11 },{ 28,  18,  11 },{ 28,  18,  11 },
{ 29,  18,  11 },{ 29,  18,  12 },{ 29,  18,  12 },{ 30,  19,  12 },{ 30,  19,  12 },
{ 30,  19,  12 },{ 31,  19,  12 },{ 31,  19,  12 },{ 31,  20,  12 },{ 32,  20,  13 },
{ 32,  20,  13 },{ 32,  20,  13 },{ 33,  20,  13 },{ 33,  21,  13 },{ 33,  21,  13 },
{ 34,  21,  13 },{ 34,  21,  13 },{ 34,  21,  14 },{ 34,  22,  14 },{ 35,  22,  14 },
{ 35,  22,  14 },{ 35,  22,  14 },{ 36,  22,  14 },{ 36,  23,  14 },{ 36,  23,  14 },
{ 37,  23,  15 },{ 37,  23,  15 },{ 37,  23,  15 },{ 38,  24,  15 },{ 38,  24,  15 },
{ 38,  24,  15 },{ 39,  24,  15 },{ 39,  24,  15 },{ 39,  25,  16 },{ 40,  25,  16 },
{ 40,  25,  16 },{ 40,  25,  16 },{ 41,  25,  16 },{ 41,  26,  16 },{ 41,  26,  16 },
{ 41,  26,  17 },{ 42,  26,  17 },{ 42,  26,  17 },{ 42,  27,  17 },{ 43,  27,  17 },
{ 43,  27,  17 },{ 43,  27,  17 },{ 44,  27,  17 },{ 44,  28,  18 },{ 44,  28,  18 },
{ 45,  28,  18 },{ 45,  28,  18 },{ 45,  28,  18 },{ 46,  29,  18 },{ 46,  29,  18 },
{ 46,  29,  18 },{ 47,  29,  19 },{ 47,  29,  19 },{ 47,  30,  19 },{ 48,  30,  19 },
{ 48,  30,  19 },{ 48,  30,  19 },{ 48,  30,  19 },{ 49,  31,  19 },{ 49,  31,  20 },
{ 49,  31,  20 },{ 50,  31,  20 },{ 50,  31,  20 },{ 50,  32,  20 },{ 51,  32,  20 },
{ 51,  32,  20 },{ 51,  32,  20 },{ 52,  32,  21 },{ 52,  33,  21 },{ 52,  33,  21 },
{ 53,  33,  21 },{ 53,  33,  21 },{ 53,  33,  21 },{ 54,  34,  21 },{ 54,  34,  21 },
{ 54,  34,  22 },{ 55,  34,  22 },{ 55,  34,  22 },{ 55,  34,  22 },{ 56,  35,  22 },
{ 56,  35,  22 },{ 56,  35,  22 },{ 56,  35,  22 },{ 57,  35,  23 },{ 57,  36,  23 },
{ 57,  36,  23 },{ 58,  36,  23 },{ 58,  36,  23 },{ 58,  36,  23 },{ 59,  37,  23 },
{ 59,  37,  23 },{ 59,  37,  24 },{ 60,  37,  24 },{ 60,  37,  24 },{ 60,  38,  24 },
{ 61,  38,  24 },{ 61,  38,  24 },{ 61,  38,  24 },{ 62,  38,  25 },{ 62,  39,  25 },
{ 62,  39,  25 },{ 63,  39,  25 },{ 63,  39,  25 },{ 63,  39,  25 },{ 63,  40,  25 },
{ 64,  40,  25 },{ 64,  40,  26 },{ 64,  40,  26 },{ 65,  40,  26 },{ 65,  41,  26 },
{ 65,  41,  26 },{ 66,  41,  26 },{ 66,  41,  26 },{ 66,  41,  26 },{ 67,  42,  27 },
{ 67,  42,  27 },{ 67,  42,  27 },{ 68,  42,  27 },{ 68,  42,  27 },{ 68,  43,  27 },
{ 69,  43,  27 },{ 69,  43,  27 },{ 69,  43,  28 },{ 70,  43,  28 },{ 70,  44,  28 },
{ 70,  44,  28 },{ 71,  44,  28 },{ 71,  44,  28 },{ 71,  44,  28 },{ 71,  45,  28 },
{ 72,  45,  29 },{ 72,  45,  29 },{ 72,  45,  29 },{ 73,  45,  29 },{ 73,  46,  29 },
{ 73,  46,  29 },{ 74,  46,  29 },{ 74,  46,  29 },{ 74,  46,  30 },{ 75,  47,  30 },
{ 75,  47,  30 },{ 75,  47,  30 },{ 76,  47,  30 },{ 76,  47,  30 },{ 76,  48,  30 },
{ 77,  48,  30 },{ 77,  48,  31 },{ 77,  48,  31 },{ 78,  48,  31 },{ 78,  49,  31 },
{ 78,  49,  31 },{ 78,  49,  31 },{ 79,  49,  31 },{ 79,  49,  31 },{ 79,  50,  32 },
{ 80,  50,  32 },{ 80,  50,  32 },{ 80,  50,  32 },{ 81,  50,  32 },{ 81,  51,  32 },
{ 81,  51,  32 },{ 82,  51,  33 },{ 82,  51,  33 },{ 82,  51,  33 },{ 83,  52,  33 },
{ 83,  52,  33 },{ 83,  52,  33 },{ 84,  52,  33 },{ 84,  52,  33 },{ 84,  53,  34 },
{ 85,  53,  34 },{ 85,  53,  34 },{ 85,  53,  34 },{ 86,  53,  34 },{ 86,  54,  34 },
{ 86,  54,  34 },{ 86,  54,  34 },{ 87,  54,  35 },{ 87,  54,  35 },{ 87,  55,  35 },
{ 88,  55,  35 },{ 88,  55,  35 },{ 88,  55,  35 },{ 89,  55,  35 },{ 89,  56,  35 },
{ 89,  56,  36 },{ 90,  56,  36 },{ 90,  56,  36 },{ 90,  56,  36 },{ 91,  57,  36 },
{ 91,  57,  36 },{ 91,  57,  36 },{ 92,  57,  36 },{ 92,  57,  37 },{ 92,  58,  37 },
{ 93,  58,  37 },{ 93,  58,  37 },{ 93,  58,  37 },{ 93,  58,  37 },{ 94,  59,  37 },
{ 94,  59,  37 },{ 94,  59,  38 },{ 95,  59,  38 },{ 95,  59,  38 },{ 95,  60,  38 },
{ 96,  60,  38 },{ 96,  60,  38 },{ 96,  60,  38 },{ 97,  60,  38 },{ 97,  61,  39 },
{ 97,  61,  39 },{ 98,  61,  39 },{ 98,  61,  39 },{ 98,  61,  39 },{ 99,  62,  39 },
{ 99,  62,  39 },{ 99,  62,  39 },{ 100,  62,  40 },{ 100,  62,  40 },{ 100,  63,  40 },
{ 101,  63,  40 },{ 101,  63,  40 },{ 101,  63,  40 },{ 101,  63,  40 },{ 102,  64,  41 },
{ 102,  64,  41 },{ 102,  64,  41 },{ 103,  64,  41 },{ 103,  64,  41 },{ 103,  65,  41 },
{ 104,  65,  41 },{ 104,  65,  41 },{ 104,  65,  42 },{ 105,  65,  42 },{ 105,  66,  42 },
{ 105,  66,  42 },{ 106,  66,  42 },{ 106,  66,  42 },{ 106,  66,  42 },{ 107,  67,  42 },
{ 107,  67,  43 },{ 107,  67,  43 },{ 108,  67,  43 },{ 108,  67,  43 },{ 108,  68,  43 },
{ 108,  68,  43 },{ 109,  68,  43 },{ 109,  68,  43 },{ 109,  68,  44 },{ 110,  69,  44 },
{ 110,  69,  44 },{ 110,  69,  44 },{ 111,  69,  44 },{ 111,  69,  44 },{ 111,  70,  44 },
{ 112,  70,  44 },{ 112,  70,  45 },{ 112,  70,  45 },{ 113,  70,  45 },{ 113,  71,  45 },
{ 113,  71,  45 },{ 114,  71,  45 },{ 114,  71,  45 },{ 114,  71,  45 },{ 115,  72,  46 },
{ 115,  72,  46 },{ 115,  72,  46 },{ 116,  72,  46 },{ 116,  72,  46 },{ 116,  73,  46 },
{ 116,  73,  46 },{ 117,  73,  46 },{ 117,  73,  47 },{ 117,  73,  47 },{ 118,  74,  47 },
{ 118,  74,  47 },{ 118,  74,  47 },{ 119,  74,  47 },{ 119,  74,  47 },{ 119,  75,  47 },
{ 120,  75,  48 },{ 120,  75,  48 },{ 120,  75,  48 },{ 121,  75,  48 },{ 121,  76,  48 },
{ 121,  76,  48 },{ 122,  76,  48 },{ 122,  76,  49 },{ 122,  76,  49 },{ 123,  77,  49 },
{ 123,  77,  49 },{ 123,  77,  49 },{ 123,  77,  49 },{ 124,  77,  49 },{ 124,  78,  49 },
{ 124,  78,  50 },{ 125,  78,  50 },{ 125,  78,  50 },{ 125,  78,  50 },{ 126,  79,  50 },
{ 126,  79,  50 },{ 126,  79,  50 },{ 127,  79,  50 },{ 127,  79,  51 },{ 127,  80,  51 },
{ 128,  80,  51 },{ 128,  80,  51 },{ 128,  80,  51 },{ 129,  80,  51 },{ 129,  81,  51 },
{ 129,  81,  51 },{ 130,  81,  52 },{ 130,  81,  52 },{ 130,  81,  52 },{ 130,  82,  52 },
{ 131,  82,  52 },{ 131,  82,  52 },{ 131,  82,  52 },{ 132,  82,  52 },{ 132,  83,  53 },
{ 132,  83,  53 },{ 133,  83,  53 },{ 133,  83,  53 },{ 133,  83,  53 },{ 134,  84,  53 },
{ 134,  84,  53 },{ 134,  84,  53 },{ 135,  84,  54 },{ 135,  84,  54 },{ 135,  85,  54 },
{ 136,  85,  54 },{ 136,  85,  54 },{ 136,  85,  54 },{ 137,  85,  54 },{ 137,  86,  54 },
{ 137,  86,  55 },{ 138,  86,  55 },{ 138,  86,  55 },{ 138,  86,  55 },{ 138,  87,  55 },
{ 139,  87,  55 },{ 139,  87,  55 },{ 139,  87,  55 },{ 140,  87,  56 },{ 140,  88,  56 },
{ 140,  88,  56 },{ 141,  88,  56 },{ 141,  88,  56 },{ 141,  88,  56 },{ 142,  89,  56 },
{ 142,  89,  57 },{ 142,  89,  57 },{ 143,  89,  57 },{ 143,  89,  57 },{ 143,  90,  57 },
{ 144,  90,  57 },{ 144,  90,  57 },{ 144,  90,  57 },{ 145,  90,  58 },{ 145,  91,  58 },
{ 145,  91,  58 },{ 145,  91,  58 },{ 146,  91,  58 },{ 146,  91,  58 },{ 146,  92,  58 },
{ 147,  92,  58 },{ 147,  92,  59 },{ 147,  92,  59 },{ 148,  92,  59 },{ 148,  93,  59 },
{ 148,  93,  59 },{ 149,  93,  59 },{ 149,  93,  59 },{ 149,  93,  59 },{ 150,  94,  60 },
{ 150,  94,  60 },{ 150,  94,  60 },{ 151,  94,  60 },{ 151,  94,  60 },{ 151,  95,  60 },
{ 152,  95,  60 },{ 152,  95,  60 },{ 152,  95,  61 },{ 153,  95,  61 },{ 153,  96,  61 },
{ 153,  96,  61 },{ 153,  96,  61 },{ 154,  96,  61 },{ 154,  96,  61 },{ 154,  97,  61 },
{ 155,  97,  62 },{ 155,  97,  62 },{ 155,  97,  62 },{ 156,  97,  62 },{ 156,  98,  62 },
{ 156,  98,  62 },{ 157,  98,  62 },{ 157,  98,  62 },{ 157,  98,  63 },{ 158,  99,  63 },
{ 158,  99,  63 },{ 158,  99,  63 },{ 159,  99,  63 },{ 159,  99,  63 },{ 159, 100,  63 },
{ 160, 100,  63 },{ 160, 100,  64 },{ 160, 100,  64 },{ 160, 100,  64 },{ 161, 101,  64 },
{ 161, 101,  64 },{ 161, 101,  64 },{ 162, 101,  64 },{ 162, 101,  65 },{ 162, 101,  65 },
{ 163, 102,  65 },{ 163, 102,  65 },{ 163, 102,  65 },{ 164, 102,  65 },{ 164, 102,  65 },
{ 164, 103,  65 },{ 165, 103,  66 },{ 165, 103,  66 },{ 165, 103,  66 },{ 166, 103,  66 },
{ 166, 104,  66 },{ 166, 104,  66 },{ 167, 104,  66 },{ 167, 104,  66 },{ 167, 104,  67 },
{ 168, 105,  67 },{ 168, 105,  67 },{ 168, 105,  67 },{ 168, 105,  67 },{ 169, 105,  67 },
{ 169, 106,  67 },{ 169, 106,  67 },{ 170, 106,  68 },{ 170, 106,  68 },{ 170, 106,  68 },
{ 171, 107,  68 },{ 171, 107,  68 },{ 171, 107,  68 },{ 172, 107,  68 },{ 172, 107,  68 },
{ 172, 108,  69 },{ 173, 108,  69 },{ 173, 108,  69 },{ 173, 108,  69 },{ 174, 108,  69 },
{ 174, 109,  69 },{ 174, 109,  69 },{ 175, 109,  69 },{ 175, 109,  70 },{ 175, 109,  70 },
{ 175, 110,  70 },{ 176, 110,  70 },{ 176, 110,  70 },{ 176, 110,  70 },{ 177, 110,  70 },
{ 177, 111,  70 },{ 177, 111,  71 },{ 178, 111,  71 },{ 178, 111,  71 },{ 178, 111,  71 },
{ 179, 112,  71 },{ 179, 112,  71 },{ 179, 112,  71 },{ 180, 112,  71 },{ 180, 112,  72 },
{ 180, 113,  72 },{ 181, 113,  72 },{ 181, 113,  72 },{ 181, 113,  72 },{ 182, 113,  72 },
{ 182, 114,  72 },{ 182, 114,  73 },{ 183, 114,  73 },{ 183, 114,  73 },{ 183, 114,  73 },
{ 183, 115,  73 },{ 184, 115,  73 },{ 184, 115,  73 },{ 184, 115,  73 },{ 185, 115,  74 },
{ 185, 116,  74 },{ 185, 116,  74 },{ 186, 116,  74 },{ 186, 116,  74 },{ 186, 116,  74 },
{ 187, 117,  74 },{ 187, 117,  74 },{ 187, 117,  75 },{ 188, 117,  75 },{ 188, 117,  75 },
{ 188, 118,  75 },{ 189, 118,  75 },{ 189, 118,  75 },{ 189, 118,  75 },{ 190, 118,  75 },
{ 190, 119,  76 },{ 190, 119,  76 },{ 190, 119,  76 },{ 191, 119,  76 },{ 191, 119,  76 },
{ 191, 120,  76 },{ 192, 120,  76 },{ 192, 120,  76 },{ 192, 120,  77 },{ 193, 120,  77 },
{ 193, 121,  77 },{ 193, 121,  77 },{ 194, 121,  77 },{ 194, 121,  77 },{ 194, 121,  77 },
{ 195, 122,  77 },{ 195, 122,  78 },{ 195, 122,  78 },{ 196, 122,  78 },{ 196, 122,  78 },
{ 196, 123,  78 },{ 197, 123,  78 },{ 197, 123,  78 },{ 197, 123,  78 },{ 198, 123,  79 },
{ 198, 124,  79 },{ 198, 124,  79 },{ 198, 124,  79 },{ 199, 124,  79 },{ 199, 124,  79 },
{ 199, 125,  79 },{ 200, 125,  79 },{ 200, 125,  80 },{ 200, 125,  80 },{ 201, 125,  80 },
{ 201, 126,  80 },{ 201, 126,  80 },{ 202, 126,  80 },{ 202, 126,  80 },{ 202, 126,  81 },
{ 203, 127,  81 },{ 203, 127,  81 },{ 203, 127,  81 },{ 204, 127,  81 },{ 204, 127,  81 },
{ 204, 128,  81 },{ 205, 128,  81 },{ 205, 128,  82 },{ 205, 128,  82 },{ 205, 128,  82 },
{ 206, 129,  82 },{ 206, 129,  82 },{ 206, 129,  82 },{ 207, 129,  82 },{ 207, 129,  82 },
{ 207, 130,  83 },{ 208, 130,  83 },{ 208, 130,  83 },{ 208, 130,  83 },{ 209, 130,  83 },
{ 209, 131,  83 },{ 209, 131,  83 },{ 210, 131,  83 },{ 210, 131,  84 },{ 210, 131,  84 },
{ 211, 132,  84 },{ 211, 132,  84 },{ 211, 132,  84 },{ 212, 132,  84 },{ 212, 132,  84 },
{ 212, 133,  84 },{ 212, 133,  85 },{ 213, 133,  85 },{ 213, 133,  85 },{ 213, 133,  85 },
{ 214, 134,  85 },{ 214, 134,  85 },{ 214, 134,  85 },{ 215, 134,  85 },{ 215, 134,  86 },
{ 215, 135,  86 },{ 216, 135,  86 },{ 216, 135,  86 },{ 216, 135,  86 },{ 217, 135,  86 },
{ 217, 136,  86 },{ 217, 136,  86 },{ 218, 136,  87 },{ 218, 136,  87 },{ 218, 136,  87 },
{ 219, 137,  87 },{ 219, 137,  87 },{ 219, 137,  87 },{ 220, 137,  87 },{ 220, 137,  87 },
{ 220, 138,  88 },{ 220, 138,  88 },{ 221, 138,  88 },{ 221, 138,  88 },{ 221, 138,  88 },
{ 222, 139,  88 },{ 222, 139,  88 },{ 222, 139,  89 },{ 223, 139,  89 },{ 223, 139,  89 },
{ 223, 140,  89 },{ 224, 140,  89 },{ 224, 140,  89 },{ 224, 140,  89 },{ 225, 140,  89 },
{ 225, 141,  90 },{ 225, 141,  90 },{ 226, 141,  90 },{ 226, 141,  90 },{ 226, 141,  90 },
{ 227, 142,  90 },{ 227, 142,  90 },{ 227, 142,  90 },{ 227, 142,  91 },{ 228, 142,  91 },
{ 228, 143,  91 },{ 228, 143,  91 },{ 229, 143,  91 },{ 229, 143,  91 },{ 229, 143,  91 },
{ 230, 144,  91 },{ 230, 144,  92 },{ 230, 144,  92 },{ 231, 144,  92 },{ 231, 144,  92 },
{ 231, 145,  92 },{ 232, 145,  92 },{ 232, 145,  92 },{ 232, 145,  92 },{ 233, 145,  93 },
{ 233, 146,  93 },{ 233, 146,  93 },{ 234, 146,  93 },{ 234, 146,  93 },{ 234, 146,  93 },
{ 235, 147,  93 },{ 235, 147,  93 },{ 235, 147,  94 },{ 235, 147,  94 },{ 236, 147,  94 },
{ 236, 148,  94 },{ 236, 148,  94 },{ 237, 148,  94 },{ 237, 148,  94 },{ 237, 148,  94 },
{ 238, 149,  95 },{ 238, 149,  95 },{ 238, 149,  95 },{ 239, 149,  95 },{ 239, 149,  95 },
{ 239, 150,  95 },{ 240, 150,  95 },{ 240, 150,  95 },{ 240, 150,  96 },{ 241, 150,  96 },
{ 241, 151,  96 },{ 241, 151,  96 },{ 242, 151,  96 },{ 242, 151,  96 },{ 242, 151,  96 },
{ 242, 152,  97 },{ 243, 152,  97 },{ 243, 152,  97 },{ 243, 152,  97 },{ 244, 152,  97 },
{ 244, 153,  97 },{ 244, 153,  97 },{ 245, 153,  97 },{ 245, 153,  98 },{ 245, 153,  98 },
{ 246, 154,  98 },{ 246, 154,  98 },{ 246, 154,  98 },{ 247, 154,  98 },{ 247, 154,  98 },
{ 247, 155,  98 },{ 248, 155,  99 },{ 248, 155,  99 },{ 248, 155,  99 },{ 249, 155,  99 },
{ 249, 156,  99 },{ 249, 156,  99 },{ 250, 156,  99 },{ 250, 156,  99 },{ 250, 156, 100 },
{ 250, 157, 100 },{ 251, 157, 100 },{ 251, 157, 100 },{ 251, 157, 100 },{ 252, 157, 100 },
{ 252, 158, 100 },{ 252, 158, 100 },{ 253, 158, 101 },{ 253, 158, 101 },{ 253, 158, 101 },
{ 253, 159, 101 },{ 253, 159, 101 },{ 254, 159, 101 },{ 254, 159, 101 },{ 254, 159, 101 },
{ 254, 160, 102 },{ 254, 160, 102 },{ 254, 160, 102 },{ 254, 160, 102 },{ 254, 160, 102 },
{ 255, 161, 102 },{ 255, 161, 102 },{ 255, 161, 102 },{ 255, 161, 103 },{ 255, 161, 103 },
{ 255, 162, 103 },{ 255, 162, 103 },{ 255, 162, 103 },{ 255, 162, 103 },{ 255, 162, 103 },
{ 255, 163, 103 },{ 255, 163, 104 },{ 255, 163, 104 },{ 255, 163, 104 },{ 255, 163, 104 },
{ 255, 164, 104 },{ 255, 164, 104 },{ 255, 164, 104 },{ 255, 164, 105 },{ 255, 164, 105 },
{ 255, 165, 105 },{ 255, 165, 105 },{ 255, 165, 105 },{ 255, 165, 105 },{ 255, 165, 105 },
{ 255, 166, 105 },{ 255, 166, 106 },{ 255, 166, 106 },{ 255, 166, 106 },{ 255, 166, 106 },
{ 255, 167, 106 },{ 255, 167, 106 },{ 255, 167, 106 },{ 255, 167, 106 },{ 255, 167, 107 },
{ 255, 168, 107 },{ 255, 168, 107 },{ 255, 168, 107 },{ 255, 168, 107 },{ 255, 168, 107 },
{ 255, 168, 107 },{ 255, 169, 107 },{ 255, 169, 108 },{ 255, 169, 108 },{ 255, 169, 108 },
{ 255, 169, 108 },{ 255, 170, 108 },{ 255, 170, 108 },{ 255, 170, 108 },{ 255, 170, 108 },
{ 255, 170, 109 },{ 255, 171, 109 },{ 255, 171, 109 },{ 255, 171, 109 },{ 255, 171, 109 },
{ 255, 171, 109 },{ 255, 172, 109 },{ 255, 172, 109 },{ 255, 172, 110 },{ 255, 172, 110 },
{ 255, 172, 110 },{ 255, 173, 110 },{ 255, 173, 110 },{ 255, 173, 110 },{ 255, 173, 110 },
{ 255, 173, 110 },{ 255, 174, 111 },{ 255, 174, 111 },{ 255, 174, 111 },{ 255, 174, 111 },
{ 255, 174, 111 },{ 255, 175, 111 },{ 255, 175, 111 },{ 255, 175, 111 },{ 255, 175, 112 },
{ 255, 175, 112 },{ 255, 176, 112 },{ 255, 176, 112 },{ 255, 176, 112 },{ 255, 176, 112 },
{ 255, 176, 112 },{ 255, 177, 113 },{ 255, 177, 113 },{ 255, 177, 113 },{ 255, 177, 113 },
{ 255, 177, 113 },{ 255, 178, 113 },{ 255, 178, 113 },{ 255, 178, 113 },{ 255, 178, 114 },
{ 255, 178, 114 },{ 255, 179, 114 },{ 255, 179, 114 },{ 255, 179, 114 },{ 255, 179, 114 },
{ 255, 179, 114 },{ 255, 180, 114 },{ 255, 180, 115 },{ 255, 180, 115 },{ 255, 180, 115 },
{ 255, 180, 115 },{ 255, 181, 115 },{ 255, 181, 115 },{ 255, 181, 115 },{ 255, 181, 115 },
{ 255, 181, 116 },{ 255, 182, 116 },{ 255, 182, 116 },{ 255, 182, 116 },{ 255, 182, 116 },
{ 255, 182, 116 },{ 255, 183, 116 },{ 255, 183, 116 },{ 255, 183, 117 },{ 255, 183, 117 },
{ 255, 183, 117 },{ 255, 184, 117 },{ 255, 184, 117 },{ 255, 184, 117 },{ 255, 184, 117 },
{ 255, 184, 117 },{ 255, 185, 118 },{ 255, 185, 118 },{ 255, 185, 118 },{ 255, 185, 118 },
{ 255, 185, 118 },{ 255, 186, 118 },{ 255, 186, 118 },{ 255, 186, 118 },{ 255, 186, 119 },
{ 255, 186, 119 },{ 255, 187, 119 },{ 255, 187, 119 },{ 255, 187, 119 },{ 255, 187, 119 },
{ 255, 187, 119 },{ 255, 188, 119 },{ 255, 188, 120 },{ 255, 188, 120 },{ 255, 188, 120 },
{ 255, 188, 120 },{ 255, 189, 120 },{ 255, 189, 120 },{ 255, 189, 120 },{ 255, 189, 121 },
{ 255, 189, 121 },{ 255, 190, 121 },{ 255, 190, 121 },{ 255, 190, 121 },{ 255, 190, 121 },
{ 255, 190, 121 },{ 255, 191, 121 },{ 255, 191, 122 },{ 255, 191, 122 },{ 255, 191, 122 },
{ 255, 191, 122 },{ 255, 192, 122 },{ 255, 192, 122 },{ 255, 192, 122 },{ 255, 192, 122 },
{ 255, 192, 123 },{ 255, 193, 123 },{ 255, 193, 123 },{ 255, 193, 123 },{ 255, 193, 123 },
{ 255, 193, 123 },{ 255, 194, 123 },{ 255, 194, 123 },{ 255, 194, 124 },{ 255, 194, 124 },
{ 255, 194, 124 },{ 255, 195, 124 },{ 255, 195, 124 },{ 255, 195, 124 },{ 255, 195, 124 },
{ 255, 195, 124 },{ 255, 196, 125 },{ 255, 196, 125 },{ 255, 196, 125 },{ 255, 196, 125 },
{ 255, 196, 125 },{ 255, 197, 125 },{ 255, 197, 125 },{ 255, 197, 125 },{ 255, 197, 126 },
{ 255, 197, 126 },{ 255, 198, 126 },{ 255, 198, 126 },{ 255, 198, 126 },{ 255, 198, 126 },
{ 255, 198, 126 },{ 255, 199, 126 },{ 255, 199, 127 },{ 255, 199, 127 },{ 255, 199, 127 }
};

const rgb_t gray_colormap[1000] = {
    { 255, 255, 255 },{ 255, 255, 255 },{ 254, 254, 254 },{ 254, 254, 254 },{ 254, 254, 254 },
{ 254, 254, 254 },{ 253, 253, 253 },{ 253, 253, 253 },{ 253, 253, 253 },{ 253, 253, 253 },
{ 252, 252, 252 },{ 252, 252, 252 },{ 252, 252, 252 },{ 252, 252, 252 },{ 251, 251, 251 },
{ 251, 251, 251 },{ 251, 251, 251 },{ 251, 251, 251 },{ 250, 250, 250 },{ 250, 250, 250 },
{ 250, 250, 250 },{ 250, 250, 250 },{ 249, 249, 249 },{ 249, 249, 249 },{ 249, 249, 249 },
{ 249, 249, 249 },{ 248, 248, 248 },{ 248, 248, 248 },{ 248, 248, 248 },{ 248, 248, 248 },
{ 247, 247, 247 },{ 247, 247, 247 },{ 247, 247, 247 },{ 247, 247, 247 },{ 246, 246, 246 },
{ 246, 246, 246 },{ 246, 246, 246 },{ 246, 246, 246 },{ 245, 245, 245 },{ 245, 245, 245 },
{ 245, 245, 245 },{ 245, 245, 245 },{ 244, 244, 244 },{ 244, 244, 244 },{ 244, 244, 244 },
{ 244, 244, 244 },{ 243, 243, 243 },{ 243, 243, 243 },{ 243, 243, 243 },{ 242, 242, 242 },
{ 242, 242, 242 },{ 242, 242, 242 },{ 242, 242, 242 },{ 241, 241, 241 },{ 241, 241, 241 },
{ 241, 241, 241 },{ 241, 241, 241 },{ 240, 240, 240 },{ 240, 240, 240 },{ 240, 240, 240 },
{ 240, 240, 240 },{ 239, 239, 239 },{ 239, 239, 239 },{ 239, 239, 239 },{ 239, 239, 239 },
{ 238, 238, 238 },{ 238, 238, 238 },{ 238, 238, 238 },{ 238, 238, 238 },{ 237, 237, 237 },
{ 237, 237, 237 },{ 237, 237, 237 },{ 237, 237, 237 },{ 236, 236, 236 },{ 236, 236, 236 },
{ 236, 236, 236 },{ 236, 236, 236 },{ 235, 235, 235 },{ 235, 235, 235 },{ 235, 235, 235 },
{ 235, 235, 235 },{ 234, 234, 234 },{ 234, 234, 234 },{ 234, 234, 234 },{ 234, 234, 234 },
{ 233, 233, 233 },{ 233, 233, 233 },{ 233, 233, 233 },{ 233, 233, 233 },{ 232, 232, 232 },
{ 232, 232, 232 },{ 232, 232, 232 },{ 232, 232, 232 },{ 231, 231, 231 },{ 231, 231, 231 },
{ 231, 231, 231 },{ 230, 230, 230 },{ 230, 230, 230 },{ 230, 230, 230 },{ 230, 230, 230 },
{ 229, 229, 229 },{ 229, 229, 229 },{ 229, 229, 229 },{ 229, 229, 229 },{ 228, 228, 228 },
{ 228, 228, 228 },{ 228, 228, 228 },{ 228, 228, 228 },{ 227, 227, 227 },{ 227, 227, 227 },
{ 227, 227, 227 },{ 227, 227, 227 },{ 226, 226, 226 },{ 226, 226, 226 },{ 226, 226, 226 },
{ 226, 226, 226 },{ 225, 225, 225 },{ 225, 225, 225 },{ 225, 225, 225 },{ 225, 225, 225 },
{ 224, 224, 224 },{ 224, 224, 224 },{ 224, 224, 224 },{ 224, 224, 224 },{ 223, 223, 223 },
{ 223, 223, 223 },{ 223, 223, 223 },{ 223, 223, 223 },{ 222, 222, 222 },{ 222, 222, 222 },
{ 222, 222, 222 },{ 222, 222, 222 },{ 221, 221, 221 },{ 221, 221, 221 },{ 221, 221, 221 },
{ 221, 221, 221 },{ 220, 220, 220 },{ 220, 220, 220 },{ 220, 220, 220 },{ 220, 220, 220 },
{ 219, 219, 219 },{ 219, 219, 219 },{ 219, 219, 219 },{ 218, 218, 218 },{ 218, 218, 218 },
{ 218, 218, 218 },{ 218, 218, 218 },{ 217, 217, 217 },{ 217, 217, 217 },{ 217, 217, 217 },
{ 217, 217, 217 },{ 216, 216, 216 },{ 216, 216, 216 },{ 216, 216, 216 },{ 216, 216, 216 },
{ 215, 215, 215 },{ 215, 215, 215 },{ 215, 215, 215 },{ 215, 215, 215 },{ 214, 214, 214 },
{ 214, 214, 214 },{ 214, 214, 214 },{ 214, 214, 214 },{ 213, 213, 213 },{ 213, 213, 213 },
{ 213, 213, 213 },{ 213, 213, 213 },{ 212, 212, 212 },{ 212, 212, 212 },{ 212, 212, 212 },
{ 212, 212, 212 },{ 211, 211, 211 },{ 211, 211, 211 },{ 211, 211, 211 },{ 211, 211, 211 },
{ 210, 210, 210 },{ 210, 210, 210 },{ 210, 210, 210 },{ 210, 210, 210 },{ 209, 209, 209 },
{ 209, 209, 209 },{ 209, 209, 209 },{ 209, 209, 209 },{ 208, 208, 208 },{ 208, 208, 208 },
{ 208, 208, 208 },{ 208, 208, 208 },{ 207, 207, 207 },{ 207, 207, 207 },{ 207, 207, 207 },
{ 207, 207, 207 },{ 206, 206, 206 },{ 206, 206, 206 },{ 206, 206, 206 },{ 205, 205, 205 },
{ 205, 205, 205 },{ 205, 205, 205 },{ 205, 205, 205 },{ 204, 204, 204 },{ 204, 204, 204 },
{ 204, 204, 204 },{ 204, 204, 204 },{ 203, 203, 203 },{ 203, 203, 203 },{ 203, 203, 203 },
{ 203, 203, 203 },{ 202, 202, 202 },{ 202, 202, 202 },{ 202, 202, 202 },{ 202, 202, 202 },
{ 201, 201, 201 },{ 201, 201, 201 },{ 201, 201, 201 },{ 201, 201, 201 },{ 200, 200, 200 },
{ 200, 200, 200 },{ 200, 200, 200 },{ 200, 200, 200 },{ 199, 199, 199 },{ 199, 199, 199 },
{ 199, 199, 199 },{ 199, 199, 199 },{ 198, 198, 198 },{ 198, 198, 198 },{ 198, 198, 198 },
{ 198, 198, 198 },{ 197, 197, 197 },{ 197, 197, 197 },{ 197, 197, 197 },{ 197, 197, 197 },
{ 196, 196, 196 },{ 196, 196, 196 },{ 196, 196, 196 },{ 196, 196, 196 },{ 195, 195, 195 },
{ 195, 195, 195 },{ 195, 195, 195 },{ 195, 195, 195 },{ 194, 194, 194 },{ 194, 194, 194 },
{ 194, 194, 194 },{ 193, 193, 193 },{ 193, 193, 193 },{ 193, 193, 193 },{ 193, 193, 193 },
{ 192, 192, 192 },{ 192, 192, 192 },{ 192, 192, 192 },{ 192, 192, 192 },{ 191, 191, 191 },
{ 191, 191, 191 },{ 191, 191, 191 },{ 191, 191, 191 },{ 190, 190, 190 },{ 190, 190, 190 },
{ 190, 190, 190 },{ 190, 190, 190 },{ 189, 189, 189 },{ 189, 189, 189 },{ 189, 189, 189 },
{ 189, 189, 189 },{ 188, 188, 188 },{ 188, 188, 188 },{ 188, 188, 188 },{ 188, 188, 188 },
{ 187, 187, 187 },{ 187, 187, 187 },{ 187, 187, 187 },{ 187, 187, 187 },{ 186, 186, 186 },
{ 186, 186, 186 },{ 186, 186, 186 },{ 186, 186, 186 },{ 185, 185, 185 },{ 185, 185, 185 },
{ 185, 185, 185 },{ 185, 185, 185 },{ 184, 184, 184 },{ 184, 184, 184 },{ 184, 184, 184 },
{ 184, 184, 184 },{ 183, 183, 183 },{ 183, 183, 183 },{ 183, 183, 183 },{ 183, 183, 183 },
{ 182, 182, 182 },{ 182, 182, 182 },{ 182, 182, 182 },{ 181, 181, 181 },{ 181, 181, 181 },
{ 181, 181, 181 },{ 181, 181, 181 },{ 180, 180, 180 },{ 180, 180, 180 },{ 180, 180, 180 },
{ 180, 180, 180 },{ 179, 179, 179 },{ 179, 179, 179 },{ 179, 179, 179 },{ 179, 179, 179 },
{ 178, 178, 178 },{ 178, 178, 178 },{ 178, 178, 178 },{ 178, 178, 178 },{ 177, 177, 177 },
{ 177, 177, 177 },{ 177, 177, 177 },{ 177, 177, 177 },{ 176, 176, 176 },{ 176, 176, 176 },
{ 176, 176, 176 },{ 176, 176, 176 },{ 175, 175, 175 },{ 175, 175, 175 },{ 175, 175, 175 },
{ 175, 175, 175 },{ 174, 174, 174 },{ 174, 174, 174 },{ 174, 174, 174 },{ 174, 174, 174 },
{ 173, 173, 173 },{ 173, 173, 173 },{ 173, 173, 173 },{ 173, 173, 173 },{ 172, 172, 172 },
{ 172, 172, 172 },{ 172, 172, 172 },{ 172, 172, 172 },{ 171, 171, 171 },{ 171, 171, 171 },
{ 171, 171, 171 },{ 171, 171, 171 },{ 170, 170, 170 },{ 170, 170, 170 },{ 170, 170, 170 },
{ 169, 169, 169 },{ 169, 169, 169 },{ 169, 169, 169 },{ 169, 169, 169 },{ 168, 168, 168 },
{ 168, 168, 168 },{ 168, 168, 168 },{ 168, 168, 168 },{ 167, 167, 167 },{ 167, 167, 167 },
{ 167, 167, 167 },{ 167, 167, 167 },{ 166, 166, 166 },{ 166, 166, 166 },{ 166, 166, 166 },
{ 166, 166, 166 },{ 165, 165, 165 },{ 165, 165, 165 },{ 165, 165, 165 },{ 165, 165, 165 },
{ 164, 164, 164 },{ 164, 164, 164 },{ 164, 164, 164 },{ 164, 164, 164 },{ 163, 163, 163 },
{ 163, 163, 163 },{ 163, 163, 163 },{ 163, 163, 163 },{ 162, 162, 162 },{ 162, 162, 162 },
{ 162, 162, 162 },{ 162, 162, 162 },{ 161, 161, 161 },{ 161, 161, 161 },{ 161, 161, 161 },
{ 161, 161, 161 },{ 160, 160, 160 },{ 160, 160, 160 },{ 160, 160, 160 },{ 160, 160, 160 },
{ 159, 159, 159 },{ 159, 159, 159 },{ 159, 159, 159 },{ 159, 159, 159 },{ 158, 158, 158 },
{ 158, 158, 158 },{ 158, 158, 158 },{ 157, 157, 157 },{ 157, 157, 157 },{ 157, 157, 157 },
{ 157, 157, 157 },{ 156, 156, 156 },{ 156, 156, 156 },{ 156, 156, 156 },{ 156, 156, 156 },
{ 155, 155, 155 },{ 155, 155, 155 },{ 155, 155, 155 },{ 155, 155, 155 },{ 154, 154, 154 },
{ 154, 154, 154 },{ 154, 154, 154 },{ 154, 154, 154 },{ 153, 153, 153 },{ 153, 153, 153 },
{ 153, 153, 153 },{ 153, 153, 153 },{ 152, 152, 152 },{ 152, 152, 152 },{ 152, 152, 152 },
{ 152, 152, 152 },{ 151, 151, 151 },{ 151, 151, 151 },{ 151, 151, 151 },{ 151, 151, 151 },
{ 150, 150, 150 },{ 150, 150, 150 },{ 150, 150, 150 },{ 150, 150, 150 },{ 149, 149, 149 },
{ 149, 149, 149 },{ 149, 149, 149 },{ 149, 149, 149 },{ 148, 148, 148 },{ 148, 148, 148 },
{ 148, 148, 148 },{ 148, 148, 148 },{ 147, 147, 147 },{ 147, 147, 147 },{ 147, 147, 147 },
{ 147, 147, 147 },{ 146, 146, 146 },{ 146, 146, 146 },{ 146, 146, 146 },{ 145, 145, 145 },
{ 145, 145, 145 },{ 145, 145, 145 },{ 145, 145, 145 },{ 144, 144, 144 },{ 144, 144, 144 },
{ 144, 144, 144 },{ 144, 144, 144 },{ 143, 143, 143 },{ 143, 143, 143 },{ 143, 143, 143 },
{ 143, 143, 143 },{ 142, 142, 142 },{ 142, 142, 142 },{ 142, 142, 142 },{ 142, 142, 142 },
{ 141, 141, 141 },{ 141, 141, 141 },{ 141, 141, 141 },{ 141, 141, 141 },{ 140, 140, 140 },
{ 140, 140, 140 },{ 140, 140, 140 },{ 140, 140, 140 },{ 139, 139, 139 },{ 139, 139, 139 },
{ 139, 139, 139 },{ 139, 139, 139 },{ 138, 138, 138 },{ 138, 138, 138 },{ 138, 138, 138 },
{ 138, 138, 138 },{ 137, 137, 137 },{ 137, 137, 137 },{ 137, 137, 137 },{ 137, 137, 137 },
{ 136, 136, 136 },{ 136, 136, 136 },{ 136, 136, 136 },{ 136, 136, 136 },{ 135, 135, 135 },
{ 135, 135, 135 },{ 135, 135, 135 },{ 135, 135, 135 },{ 134, 134, 134 },{ 134, 134, 134 },
{ 134, 134, 134 },{ 133, 133, 133 },{ 133, 133, 133 },{ 133, 133, 133 },{ 133, 133, 133 },
{ 132, 132, 132 },{ 132, 132, 132 },{ 132, 132, 132 },{ 132, 132, 132 },{ 131, 131, 131 },
{ 131, 131, 131 },{ 131, 131, 131 },{ 131, 131, 131 },{ 130, 130, 130 },{ 130, 130, 130 },
{ 130, 130, 130 },{ 130, 130, 130 },{ 129, 129, 129 },{ 129, 129, 129 },{ 129, 129, 129 },
{ 129, 129, 129 },{ 128, 128, 128 },{ 128, 128, 128 },{ 128, 128, 128 },{ 128, 128, 128 },
{ 127, 127, 127 },{ 127, 127, 127 },{ 127, 127, 127 },{ 127, 127, 127 },{ 126, 126, 126 },
{ 126, 126, 126 },{ 126, 126, 126 },{ 126, 126, 126 },{ 125, 125, 125 },{ 125, 125, 125 },
{ 125, 125, 125 },{ 125, 125, 125 },{ 124, 124, 124 },{ 124, 124, 124 },{ 124, 124, 124 },
{ 124, 124, 124 },{ 123, 123, 123 },{ 123, 123, 123 },{ 123, 123, 123 },{ 123, 123, 123 },
{ 122, 122, 122 },{ 122, 122, 122 },{ 122, 122, 122 },{ 122, 122, 122 },{ 121, 121, 121 },
{ 121, 121, 121 },{ 121, 121, 121 },{ 120, 120, 120 },{ 120, 120, 120 },{ 120, 120, 120 },
{ 120, 120, 120 },{ 119, 119, 119 },{ 119, 119, 119 },{ 119, 119, 119 },{ 119, 119, 119 },
{ 118, 118, 118 },{ 118, 118, 118 },{ 118, 118, 118 },{ 118, 118, 118 },{ 117, 117, 117 },
{ 117, 117, 117 },{ 117, 117, 117 },{ 117, 117, 117 },{ 116, 116, 116 },{ 116, 116, 116 },
{ 116, 116, 116 },{ 116, 116, 116 },{ 115, 115, 115 },{ 115, 115, 115 },{ 115, 115, 115 },
{ 115, 115, 115 },{ 114, 114, 114 },{ 114, 114, 114 },{ 114, 114, 114 },{ 114, 114, 114 },
{ 113, 113, 113 },{ 113, 113, 113 },{ 113, 113, 113 },{ 113, 113, 113 },{ 112, 112, 112 },
{ 112, 112, 112 },{ 112, 112, 112 },{ 112, 112, 112 },{ 111, 111, 111 },{ 111, 111, 111 },
{ 111, 111, 111 },{ 111, 111, 111 },{ 110, 110, 110 },{ 110, 110, 110 },{ 110, 110, 110 },
{ 110, 110, 110 },{ 109, 109, 109 },{ 109, 109, 109 },{ 109, 109, 109 },{ 108, 108, 108 },
{ 108, 108, 108 },{ 108, 108, 108 },{ 108, 108, 108 },{ 107, 107, 107 },{ 107, 107, 107 },
{ 107, 107, 107 },{ 107, 107, 107 },{ 106, 106, 106 },{ 106, 106, 106 },{ 106, 106, 106 },
{ 106, 106, 106 },{ 105, 105, 105 },{ 105, 105, 105 },{ 105, 105, 105 },{ 105, 105, 105 },
{ 104, 104, 104 },{ 104, 104, 104 },{ 104, 104, 104 },{ 104, 104, 104 },{ 103, 103, 103 },
{ 103, 103, 103 },{ 103, 103, 103 },{ 103, 103, 103 },{ 102, 102, 102 },{ 102, 102, 102 },
{ 102, 102, 102 },{ 102, 102, 102 },{ 101, 101, 101 },{ 101, 101, 101 },{ 101, 101, 101 },
{ 101, 101, 101 },{ 100, 100, 100 },{ 100, 100, 100 },{ 100, 100, 100 },{ 100, 100, 100 },
{ 99,  99,  99 },{ 99,  99,  99 },{ 99,  99,  99 },{ 99,  99,  99 },{ 98,  98,  98 },
{ 98,  98,  98 },{ 98,  98,  98 },{ 98,  98,  98 },{ 97,  97,  97 },{ 97,  97,  97 },
{ 97,  97,  97 },{ 96,  96,  96 },{ 96,  96,  96 },{ 96,  96,  96 },{ 96,  96,  96 },
{ 95,  95,  95 },{ 95,  95,  95 },{ 95,  95,  95 },{ 95,  95,  95 },{ 94,  94,  94 },
{ 94,  94,  94 },{ 94,  94,  94 },{ 94,  94,  94 },{ 93,  93,  93 },{ 93,  93,  93 },
{ 93,  93,  93 },{ 93,  93,  93 },{ 92,  92,  92 },{ 92,  92,  92 },{ 92,  92,  92 },
{ 92,  92,  92 },{ 91,  91,  91 },{ 91,  91,  91 },{ 91,  91,  91 },{ 91,  91,  91 },
{ 90,  90,  90 },{ 90,  90,  90 },{ 90,  90,  90 },{ 90,  90,  90 },{ 89,  89,  89 },
{ 89,  89,  89 },{ 89,  89,  89 },{ 89,  89,  89 },{ 88,  88,  88 },{ 88,  88,  88 },
{ 88,  88,  88 },{ 88,  88,  88 },{ 87,  87,  87 },{ 87,  87,  87 },{ 87,  87,  87 },
{ 87,  87,  87 },{ 86,  86,  86 },{ 86,  86,  86 },{ 86,  86,  86 },{ 86,  86,  86 },
{ 85,  85,  85 },{ 85,  85,  85 },{ 85,  85,  85 },{ 84,  84,  84 },{ 84,  84,  84 },
{ 84,  84,  84 },{ 84,  84,  84 },{ 83,  83,  83 },{ 83,  83,  83 },{ 83,  83,  83 },
{ 83,  83,  83 },{ 82,  82,  82 },{ 82,  82,  82 },{ 82,  82,  82 },{ 82,  82,  82 },
{ 81,  81,  81 },{ 81,  81,  81 },{ 81,  81,  81 },{ 81,  81,  81 },{ 80,  80,  80 },
{ 80,  80,  80 },{ 80,  80,  80 },{ 80,  80,  80 },{ 79,  79,  79 },{ 79,  79,  79 },
{ 79,  79,  79 },{ 79,  79,  79 },{ 78,  78,  78 },{ 78,  78,  78 },{ 78,  78,  78 },
{ 78,  78,  78 },{ 77,  77,  77 },{ 77,  77,  77 },{ 77,  77,  77 },{ 77,  77,  77 },
{ 76,  76,  76 },{ 76,  76,  76 },{ 76,  76,  76 },{ 76,  76,  76 },{ 75,  75,  75 },
{ 75,  75,  75 },{ 75,  75,  75 },{ 75,  75,  75 },{ 74,  74,  74 },{ 74,  74,  74 },
{ 74,  74,  74 },{ 74,  74,  74 },{ 73,  73,  73 },{ 73,  73,  73 },{ 73,  73,  73 },
{ 72,  72,  72 },{ 72,  72,  72 },{ 72,  72,  72 },{ 72,  72,  72 },{ 71,  71,  71 },
{ 71,  71,  71 },{ 71,  71,  71 },{ 71,  71,  71 },{ 70,  70,  70 },{ 70,  70,  70 },
{ 70,  70,  70 },{ 70,  70,  70 },{ 69,  69,  69 },{ 69,  69,  69 },{ 69,  69,  69 },
{ 69,  69,  69 },{ 68,  68,  68 },{ 68,  68,  68 },{ 68,  68,  68 },{ 68,  68,  68 },
{ 67,  67,  67 },{ 67,  67,  67 },{ 67,  67,  67 },{ 67,  67,  67 },{ 66,  66,  66 },
{ 66,  66,  66 },{ 66,  66,  66 },{ 66,  66,  66 },{ 65,  65,  65 },{ 65,  65,  65 },
{ 65,  65,  65 },{ 65,  65,  65 },{ 64,  64,  64 },{ 64,  64,  64 },{ 64,  64,  64 },
{ 64,  64,  64 },{ 63,  63,  63 },{ 63,  63,  63 },{ 63,  63,  63 },{ 63,  63,  63 },
{ 62,  62,  62 },{ 62,  62,  62 },{ 62,  62,  62 },{ 62,  62,  62 },{ 61,  61,  61 },
{ 61,  61,  61 },{ 61,  61,  61 },{ 60,  60,  60 },{ 60,  60,  60 },{ 60,  60,  60 },
{ 60,  60,  60 },{ 59,  59,  59 },{ 59,  59,  59 },{ 59,  59,  59 },{ 59,  59,  59 },
{ 58,  58,  58 },{ 58,  58,  58 },{ 58,  58,  58 },{ 58,  58,  58 },{ 57,  57,  57 },
{ 57,  57,  57 },{ 57,  57,  57 },{ 57,  57,  57 },{ 56,  56,  56 },{ 56,  56,  56 },
{ 56,  56,  56 },{ 56,  56,  56 },{ 55,  55,  55 },{ 55,  55,  55 },{ 55,  55,  55 },
{ 55,  55,  55 },{ 54,  54,  54 },{ 54,  54,  54 },{ 54,  54,  54 },{ 54,  54,  54 },
{ 53,  53,  53 },{ 53,  53,  53 },{ 53,  53,  53 },{ 53,  53,  53 },{ 52,  52,  52 },
{ 52,  52,  52 },{ 52,  52,  52 },{ 52,  52,  52 },{ 51,  51,  51 },{ 51,  51,  51 },
{ 51,  51,  51 },{ 51,  51,  51 },{ 50,  50,  50 },{ 50,  50,  50 },{ 50,  50,  50 },
{ 50,  50,  50 },{ 49,  49,  49 },{ 49,  49,  49 },{ 49,  49,  49 },{ 48,  48,  48 },
{ 48,  48,  48 },{ 48,  48,  48 },{ 48,  48,  48 },{ 47,  47,  47 },{ 47,  47,  47 },
{ 47,  47,  47 },{ 47,  47,  47 },{ 46,  46,  46 },{ 46,  46,  46 },{ 46,  46,  46 },
{ 46,  46,  46 },{ 45,  45,  45 },{ 45,  45,  45 },{ 45,  45,  45 },{ 45,  45,  45 },
{ 44,  44,  44 },{ 44,  44,  44 },{ 44,  44,  44 },{ 44,  44,  44 },{ 43,  43,  43 },
{ 43,  43,  43 },{ 43,  43,  43 },{ 43,  43,  43 },{ 42,  42,  42 },{ 42,  42,  42 },
{ 42,  42,  42 },{ 42,  42,  42 },{ 41,  41,  41 },{ 41,  41,  41 },{ 41,  41,  41 },
{ 41,  41,  41 },{ 40,  40,  40 },{ 40,  40,  40 },{ 40,  40,  40 },{ 40,  40,  40 },
{ 39,  39,  39 },{ 39,  39,  39 },{ 39,  39,  39 },{ 39,  39,  39 },{ 38,  38,  38 },
{ 38,  38,  38 },{ 38,  38,  38 },{ 38,  38,  38 },{ 37,  37,  37 },{ 37,  37,  37 },
{ 37,  37,  37 },{ 37,  37,  37 },{ 36,  36,  36 },{ 36,  36,  36 },{ 36,  36,  36 },
{ 35,  35,  35 },{ 35,  35,  35 },{ 35,  35,  35 },{ 35,  35,  35 },{ 34,  34,  34 },
{ 34,  34,  34 },{ 34,  34,  34 },{ 34,  34,  34 },{ 33,  33,  33 },{ 33,  33,  33 },
{ 33,  33,  33 },{ 33,  33,  33 },{ 32,  32,  32 },{ 32,  32,  32 },{ 32,  32,  32 },
{ 32,  32,  32 },{ 31,  31,  31 },{ 31,  31,  31 },{ 31,  31,  31 },{ 31,  31,  31 },
{ 30,  30,  30 },{ 30,  30,  30 },{ 30,  30,  30 },{ 30,  30,  30 },{ 29,  29,  29 },
{ 29,  29,  29 },{ 29,  29,  29 },{ 29,  29,  29 },{ 28,  28,  28 },{ 28,  28,  28 },
{ 28,  28,  28 },{ 28,  28,  28 },{ 27,  27,  27 },{ 27,  27,  27 },{ 27,  27,  27 },
{ 27,  27,  27 },{ 26,  26,  26 },{ 26,  26,  26 },{ 26,  26,  26 },{ 26,  26,  26 },
{ 25,  25,  25 },{ 25,  25,  25 },{ 25,  25,  25 },{ 25,  25,  25 },{ 24,  24,  24 },
{ 24,  24,  24 },{ 24,  24,  24 },{ 23,  23,  23 },{ 23,  23,  23 },{ 23,  23,  23 },
{ 23,  23,  23 },{ 22,  22,  22 },{ 22,  22,  22 },{ 22,  22,  22 },{ 22,  22,  22 },
{ 21,  21,  21 },{ 21,  21,  21 },{ 21,  21,  21 },{ 21,  21,  21 },{ 20,  20,  20 },
{ 20,  20,  20 },{ 20,  20,  20 },{ 20,  20,  20 },{ 19,  19,  19 },{ 19,  19,  19 },
{ 19,  19,  19 },{ 19,  19,  19 },{ 18,  18,  18 },{ 18,  18,  18 },{ 18,  18,  18 },
{ 18,  18,  18 },{ 17,  17,  17 },{ 17,  17,  17 },{ 17,  17,  17 },{ 17,  17,  17 },
{ 16,  16,  16 },{ 16,  16,  16 },{ 16,  16,  16 },{ 16,  16,  16 },{ 15,  15,  15 },
{ 15,  15,  15 },{ 15,  15,  15 },{ 15,  15,  15 },{ 14,  14,  14 },{ 14,  14,  14 },
{ 14,  14,  14 },{ 14,  14,  14 },{ 13,  13,  13 },{ 13,  13,  13 },{ 13,  13,  13 },
{ 13,  13,  13 },{ 12,  12,  12 },{ 12,  12,  12 },{ 12,  12,  12 },{ 11,  11,  11 },
{ 11,  11,  11 },{ 11,  11,  11 },{ 11,  11,  11 },{ 10,  10,  10 },{ 10,  10,  10 },
{ 10,  10,  10 },{ 10,  10,  10 },{ 9,   9,   9 },{ 9,   9,   9 },{ 9,   9,   9 },
{ 9,   9,   9 },{ 8,   8,   8 },{ 8,   8,   8 },{ 8,   8,   8 },{ 8,   8,   8 },
{ 7,   7,   7 },{ 7,   7,   7 },{ 7,   7,   7 },{ 7,   7,   7 },{ 6,   6,   6 },
{ 6,   6,   6 },{ 6,   6,   6 },{ 6,   6,   6 },{ 5,   5,   5 },{ 5,   5,   5 },
{ 5,   5,   5 },{ 5,   5,   5 },{ 4,   4,   4 },{ 4,   4,   4 },{ 4,   4,   4 },
{ 4,   4,   4 },{ 3,   3,   3 },{ 3,   3,   3 },{ 3,   3,   3 },{ 3,   3,   3 },
{ 2,   2,   2 },{ 2,   2,   2 },{ 2,   2,   2 },{ 2,   2,   2 },{ 1,   1,   1 },
{ 1,   1,   1 },{ 1,   1,   1 },{ 1,   1,   1 },{ 0,   0,   0 },{ 0,   0,   0 }
};

const rgb_t hot_colormap[1000] = {
    { 11,   0,   0 },{ 11,   0,   0 },{ 12,   0,   0 },{ 13,   0,   0 },{ 13,   0,   0 },
{ 14,   0,   0 },{ 15,   0,   0 },{ 15,   0,   0 },{ 16,   0,   0 },{ 17,   0,   0 },
{ 17,   0,   0 },{ 18,   0,   0 },{ 19,   0,   0 },{ 19,   0,   0 },{ 20,   0,   0 },
{ 21,   0,   0 },{ 21,   0,   0 },{ 22,   0,   0 },{ 23,   0,   0 },{ 23,   0,   0 },
{ 24,   0,   0 },{ 25,   0,   0 },{ 25,   0,   0 },{ 26,   0,   0 },{ 27,   0,   0 },
{ 27,   0,   0 },{ 28,   0,   0 },{ 29,   0,   0 },{ 29,   0,   0 },{ 30,   0,   0 },
{ 31,   0,   0 },{ 31,   0,   0 },{ 32,   0,   0 },{ 33,   0,   0 },{ 33,   0,   0 },
{ 34,   0,   0 },{ 35,   0,   0 },{ 35,   0,   0 },{ 36,   0,   0 },{ 37,   0,   0 },
{ 37,   0,   0 },{ 38,   0,   0 },{ 39,   0,   0 },{ 39,   0,   0 },{ 40,   0,   0 },
{ 41,   0,   0 },{ 41,   0,   0 },{ 42,   0,   0 },{ 43,   0,   0 },{ 43,   0,   0 },
{ 44,   0,   0 },{ 45,   0,   0 },{ 45,   0,   0 },{ 46,   0,   0 },{ 47,   0,   0 },
{ 47,   0,   0 },{ 48,   0,   0 },{ 49,   0,   0 },{ 49,   0,   0 },{ 50,   0,   0 },
{ 51,   0,   0 },{ 51,   0,   0 },{ 52,   0,   0 },{ 53,   0,   0 },{ 54,   0,   0 },
{ 54,   0,   0 },{ 55,   0,   0 },{ 56,   0,   0 },{ 56,   0,   0 },{ 57,   0,   0 },
{ 58,   0,   0 },{ 58,   0,   0 },{ 59,   0,   0 },{ 60,   0,   0 },{ 60,   0,   0 },
{ 61,   0,   0 },{ 62,   0,   0 },{ 62,   0,   0 },{ 63,   0,   0 },{ 64,   0,   0 },
{ 64,   0,   0 },{ 65,   0,   0 },{ 66,   0,   0 },{ 66,   0,   0 },{ 67,   0,   0 },
{ 68,   0,   0 },{ 68,   0,   0 },{ 69,   0,   0 },{ 70,   0,   0 },{ 70,   0,   0 },
{ 71,   0,   0 },{ 72,   0,   0 },{ 72,   0,   0 },{ 73,   0,   0 },{ 74,   0,   0 },
{ 74,   0,   0 },{ 75,   0,   0 },{ 76,   0,   0 },{ 76,   0,   0 },{ 77,   0,   0 },
{ 78,   0,   0 },{ 78,   0,   0 },{ 79,   0,   0 },{ 80,   0,   0 },{ 80,   0,   0 },
{ 81,   0,   0 },{ 82,   0,   0 },{ 82,   0,   0 },{ 83,   0,   0 },{ 84,   0,   0 },
{ 84,   0,   0 },{ 85,   0,   0 },{ 86,   0,   0 },{ 86,   0,   0 },{ 87,   0,   0 },
{ 88,   0,   0 },{ 88,   0,   0 },{ 89,   0,   0 },{ 90,   0,   0 },{ 90,   0,   0 },
{ 91,   0,   0 },{ 92,   0,   0 },{ 92,   0,   0 },{ 93,   0,   0 },{ 94,   0,   0 },
{ 94,   0,   0 },{ 95,   0,   0 },{ 96,   0,   0 },{ 96,   0,   0 },{ 97,   0,   0 },
{ 98,   0,   0 },{ 98,   0,   0 },{ 99,   0,   0 },{ 100,   0,   0 },{ 100,   0,   0 },
{ 101,   0,   0 },{ 102,   0,   0 },{ 102,   0,   0 },{ 103,   0,   0 },{ 104,   0,   0 },
{ 104,   0,   0 },{ 105,   0,   0 },{ 106,   0,   0 },{ 106,   0,   0 },{ 107,   0,   0 },
{ 108,   0,   0 },{ 108,   0,   0 },{ 109,   0,   0 },{ 110,   0,   0 },{ 110,   0,   0 },
{ 111,   0,   0 },{ 112,   0,   0 },{ 112,   0,   0 },{ 113,   0,   0 },{ 114,   0,   0 },
{ 114,   0,   0 },{ 115,   0,   0 },{ 116,   0,   0 },{ 116,   0,   0 },{ 117,   0,   0 },
{ 118,   0,   0 },{ 119,   0,   0 },{ 119,   0,   0 },{ 120,   0,   0 },{ 121,   0,   0 },
{ 121,   0,   0 },{ 122,   0,   0 },{ 123,   0,   0 },{ 123,   0,   0 },{ 124,   0,   0 },
{ 125,   0,   0 },{ 125,   0,   0 },{ 126,   0,   0 },{ 127,   0,   0 },{ 127,   0,   0 },
{ 128,   0,   0 },{ 129,   0,   0 },{ 129,   0,   0 },{ 130,   0,   0 },{ 131,   0,   0 },
{ 131,   0,   0 },{ 132,   0,   0 },{ 133,   0,   0 },{ 133,   0,   0 },{ 134,   0,   0 },
{ 135,   0,   0 },{ 135,   0,   0 },{ 136,   0,   0 },{ 137,   0,   0 },{ 137,   0,   0 },
{ 138,   0,   0 },{ 139,   0,   0 },{ 139,   0,   0 },{ 140,   0,   0 },{ 141,   0,   0 },
{ 141,   0,   0 },{ 142,   0,   0 },{ 143,   0,   0 },{ 143,   0,   0 },{ 144,   0,   0 },
{ 145,   0,   0 },{ 145,   0,   0 },{ 146,   0,   0 },{ 147,   0,   0 },{ 147,   0,   0 },
{ 148,   0,   0 },{ 149,   0,   0 },{ 149,   0,   0 },{ 150,   0,   0 },{ 151,   0,   0 },
{ 151,   0,   0 },{ 152,   0,   0 },{ 153,   0,   0 },{ 153,   0,   0 },{ 154,   0,   0 },
{ 155,   0,   0 },{ 155,   0,   0 },{ 156,   0,   0 },{ 157,   0,   0 },{ 157,   0,   0 },
{ 158,   0,   0 },{ 159,   0,   0 },{ 159,   0,   0 },{ 160,   0,   0 },{ 161,   0,   0 },
{ 161,   0,   0 },{ 162,   0,   0 },{ 163,   0,   0 },{ 163,   0,   0 },{ 164,   0,   0 },
{ 165,   0,   0 },{ 165,   0,   0 },{ 166,   0,   0 },{ 167,   0,   0 },{ 167,   0,   0 },
{ 168,   0,   0 },{ 169,   0,   0 },{ 169,   0,   0 },{ 170,   0,   0 },{ 171,   0,   0 },
{ 171,   0,   0 },{ 172,   0,   0 },{ 173,   0,   0 },{ 173,   0,   0 },{ 174,   0,   0 },
{ 175,   0,   0 },{ 175,   0,   0 },{ 176,   0,   0 },{ 177,   0,   0 },{ 177,   0,   0 },
{ 178,   0,   0 },{ 179,   0,   0 },{ 179,   0,   0 },{ 180,   0,   0 },{ 181,   0,   0 },
{ 181,   0,   0 },{ 182,   0,   0 },{ 183,   0,   0 },{ 183,   0,   0 },{ 184,   0,   0 },
{ 185,   0,   0 },{ 186,   0,   0 },{ 186,   0,   0 },{ 187,   0,   0 },{ 188,   0,   0 },
{ 188,   0,   0 },{ 189,   0,   0 },{ 190,   0,   0 },{ 190,   0,   0 },{ 191,   0,   0 },
{ 192,   0,   0 },{ 192,   0,   0 },{ 193,   0,   0 },{ 194,   0,   0 },{ 194,   0,   0 },
{ 195,   0,   0 },{ 196,   0,   0 },{ 196,   0,   0 },{ 197,   0,   0 },{ 198,   0,   0 },
{ 198,   0,   0 },{ 199,   0,   0 },{ 200,   0,   0 },{ 200,   0,   0 },{ 201,   0,   0 },
{ 202,   0,   0 },{ 202,   0,   0 },{ 203,   0,   0 },{ 204,   0,   0 },{ 204,   0,   0 },
{ 205,   0,   0 },{ 206,   0,   0 },{ 206,   0,   0 },{ 207,   0,   0 },{ 208,   0,   0 },
{ 208,   0,   0 },{ 209,   0,   0 },{ 210,   0,   0 },{ 210,   0,   0 },{ 211,   0,   0 },
{ 212,   0,   0 },{ 212,   0,   0 },{ 213,   0,   0 },{ 214,   0,   0 },{ 214,   0,   0 },
{ 215,   0,   0 },{ 216,   0,   0 },{ 216,   0,   0 },{ 217,   0,   0 },{ 218,   0,   0 },
{ 218,   0,   0 },{ 219,   0,   0 },{ 220,   0,   0 },{ 220,   0,   0 },{ 221,   0,   0 },
{ 222,   0,   0 },{ 222,   0,   0 },{ 223,   0,   0 },{ 224,   0,   0 },{ 224,   0,   0 },
{ 225,   0,   0 },{ 226,   0,   0 },{ 226,   0,   0 },{ 227,   0,   0 },{ 228,   0,   0 },
{ 228,   0,   0 },{ 229,   0,   0 },{ 230,   0,   0 },{ 230,   0,   0 },{ 231,   0,   0 },
{ 232,   0,   0 },{ 232,   0,   0 },{ 233,   0,   0 },{ 234,   0,   0 },{ 234,   0,   0 },
{ 235,   0,   0 },{ 236,   0,   0 },{ 236,   0,   0 },{ 237,   0,   0 },{ 238,   0,   0 },
{ 238,   0,   0 },{ 239,   0,   0 },{ 240,   0,   0 },{ 240,   0,   0 },{ 241,   0,   0 },
{ 242,   0,   0 },{ 242,   0,   0 },{ 243,   0,   0 },{ 244,   0,   0 },{ 244,   0,   0 },
{ 245,   0,   0 },{ 246,   0,   0 },{ 246,   0,   0 },{ 247,   0,   0 },{ 248,   0,   0 },
{ 248,   0,   0 },{ 249,   0,   0 },{ 250,   0,   0 },{ 251,   0,   0 },{ 251,   0,   0 },
{ 252,   0,   0 },{ 253,   0,   0 },{ 253,   0,   0 },{ 254,   0,   0 },{ 255,   0,   0 },
{ 255,   0,   0 },{ 255,   1,   0 },{ 255,   2,   0 },{ 255,   2,   0 },{ 255,   3,   0 },
{ 255,   4,   0 },{ 255,   4,   0 },{ 255,   5,   0 },{ 255,   6,   0 },{ 255,   6,   0 },
{ 255,   7,   0 },{ 255,   8,   0 },{ 255,   8,   0 },{ 255,   9,   0 },{ 255,  10,   0 },
{ 255,  10,   0 },{ 255,  11,   0 },{ 255,  12,   0 },{ 255,  12,   0 },{ 255,  13,   0 },
{ 255,  14,   0 },{ 255,  14,   0 },{ 255,  15,   0 },{ 255,  16,   0 },{ 255,  16,   0 },
{ 255,  17,   0 },{ 255,  18,   0 },{ 255,  18,   0 },{ 255,  19,   0 },{ 255,  20,   0 },
{ 255,  20,   0 },{ 255,  21,   0 },{ 255,  22,   0 },{ 255,  22,   0 },{ 255,  23,   0 },
{ 255,  24,   0 },{ 255,  24,   0 },{ 255,  25,   0 },{ 255,  26,   0 },{ 255,  26,   0 },
{ 255,  27,   0 },{ 255,  28,   0 },{ 255,  28,   0 },{ 255,  29,   0 },{ 255,  30,   0 },
{ 255,  30,   0 },{ 255,  31,   0 },{ 255,  32,   0 },{ 255,  32,   0 },{ 255,  33,   0 },
{ 255,  34,   0 },{ 255,  34,   0 },{ 255,  35,   0 },{ 255,  36,   0 },{ 255,  36,   0 },
{ 255,  37,   0 },{ 255,  38,   0 },{ 255,  38,   0 },{ 255,  39,   0 },{ 255,  40,   0 },
{ 255,  40,   0 },{ 255,  41,   0 },{ 255,  42,   0 },{ 255,  42,   0 },{ 255,  43,   0 },
{ 255,  44,   0 },{ 255,  44,   0 },{ 255,  45,   0 },{ 255,  46,   0 },{ 255,  46,   0 },
{ 255,  47,   0 },{ 255,  48,   0 },{ 255,  48,   0 },{ 255,  49,   0 },{ 255,  50,   0 },
{ 255,  50,   0 },{ 255,  51,   0 },{ 255,  52,   0 },{ 255,  52,   0 },{ 255,  53,   0 },
{ 255,  54,   0 },{ 255,  54,   0 },{ 255,  55,   0 },{ 255,  56,   0 },{ 255,  56,   0 },
{ 255,  57,   0 },{ 255,  58,   0 },{ 255,  58,   0 },{ 255,  59,   0 },{ 255,  60,   0 },
{ 255,  60,   0 },{ 255,  61,   0 },{ 255,  62,   0 },{ 255,  63,   0 },{ 255,  63,   0 },
{ 255,  64,   0 },{ 255,  65,   0 },{ 255,  65,   0 },{ 255,  66,   0 },{ 255,  67,   0 },
{ 255,  67,   0 },{ 255,  68,   0 },{ 255,  69,   0 },{ 255,  69,   0 },{ 255,  70,   0 },
{ 255,  71,   0 },{ 255,  71,   0 },{ 255,  72,   0 },{ 255,  73,   0 },{ 255,  73,   0 },
{ 255,  74,   0 },{ 255,  75,   0 },{ 255,  75,   0 },{ 255,  76,   0 },{ 255,  77,   0 },
{ 255,  77,   0 },{ 255,  78,   0 },{ 255,  79,   0 },{ 255,  79,   0 },{ 255,  80,   0 },
{ 255,  81,   0 },{ 255,  81,   0 },{ 255,  82,   0 },{ 255,  83,   0 },{ 255,  83,   0 },
{ 255,  84,   0 },{ 255,  85,   0 },{ 255,  85,   0 },{ 255,  86,   0 },{ 255,  87,   0 },
{ 255,  87,   0 },{ 255,  88,   0 },{ 255,  89,   0 },{ 255,  89,   0 },{ 255,  90,   0 },
{ 255,  91,   0 },{ 255,  91,   0 },{ 255,  92,   0 },{ 255,  93,   0 },{ 255,  93,   0 },
{ 255,  94,   0 },{ 255,  95,   0 },{ 255,  95,   0 },{ 255,  96,   0 },{ 255,  97,   0 },
{ 255,  97,   0 },{ 255,  98,   0 },{ 255,  99,   0 },{ 255,  99,   0 },{ 255, 100,   0 },
{ 255, 101,   0 },{ 255, 101,   0 },{ 255, 102,   0 },{ 255, 103,   0 },{ 255, 103,   0 },
{ 255, 104,   0 },{ 255, 105,   0 },{ 255, 105,   0 },{ 255, 106,   0 },{ 255, 107,   0 },
{ 255, 107,   0 },{ 255, 108,   0 },{ 255, 109,   0 },{ 255, 109,   0 },{ 255, 110,   0 },
{ 255, 111,   0 },{ 255, 111,   0 },{ 255, 112,   0 },{ 255, 113,   0 },{ 255, 113,   0 },
{ 255, 114,   0 },{ 255, 115,   0 },{ 255, 115,   0 },{ 255, 116,   0 },{ 255, 117,   0 },
{ 255, 117,   0 },{ 255, 118,   0 },{ 255, 119,   0 },{ 255, 119,   0 },{ 255, 120,   0 },
{ 255, 121,   0 },{ 255, 121,   0 },{ 255, 122,   0 },{ 255, 123,   0 },{ 255, 123,   0 },
{ 255, 124,   0 },{ 255, 125,   0 },{ 255, 125,   0 },{ 255, 126,   0 },{ 255, 127,   0 },
{ 255, 128,   0 },{ 255, 128,   0 },{ 255, 129,   0 },{ 255, 130,   0 },{ 255, 130,   0 },
{ 255, 131,   0 },{ 255, 132,   0 },{ 255, 132,   0 },{ 255, 133,   0 },{ 255, 134,   0 },
{ 255, 134,   0 },{ 255, 135,   0 },{ 255, 136,   0 },{ 255, 136,   0 },{ 255, 137,   0 },
{ 255, 138,   0 },{ 255, 138,   0 },{ 255, 139,   0 },{ 255, 140,   0 },{ 255, 140,   0 },
{ 255, 141,   0 },{ 255, 142,   0 },{ 255, 142,   0 },{ 255, 143,   0 },{ 255, 144,   0 },
{ 255, 144,   0 },{ 255, 145,   0 },{ 255, 146,   0 },{ 255, 146,   0 },{ 255, 147,   0 },
{ 255, 148,   0 },{ 255, 148,   0 },{ 255, 149,   0 },{ 255, 150,   0 },{ 255, 150,   0 },
{ 255, 151,   0 },{ 255, 152,   0 },{ 255, 152,   0 },{ 255, 153,   0 },{ 255, 154,   0 },
{ 255, 154,   0 },{ 255, 155,   0 },{ 255, 156,   0 },{ 255, 156,   0 },{ 255, 157,   0 },
{ 255, 158,   0 },{ 255, 158,   0 },{ 255, 159,   0 },{ 255, 160,   0 },{ 255, 160,   0 },
{ 255, 161,   0 },{ 255, 162,   0 },{ 255, 162,   0 },{ 255, 163,   0 },{ 255, 164,   0 },
{ 255, 164,   0 },{ 255, 165,   0 },{ 255, 166,   0 },{ 255, 166,   0 },{ 255, 167,   0 },
{ 255, 168,   0 },{ 255, 168,   0 },{ 255, 169,   0 },{ 255, 170,   0 },{ 255, 170,   0 },
{ 255, 171,   0 },{ 255, 172,   0 },{ 255, 172,   0 },{ 255, 173,   0 },{ 255, 174,   0 },
{ 255, 174,   0 },{ 255, 175,   0 },{ 255, 176,   0 },{ 255, 176,   0 },{ 255, 177,   0 },
{ 255, 178,   0 },{ 255, 178,   0 },{ 255, 179,   0 },{ 255, 180,   0 },{ 255, 180,   0 },
{ 255, 181,   0 },{ 255, 182,   0 },{ 255, 182,   0 },{ 255, 183,   0 },{ 255, 184,   0 },
{ 255, 184,   0 },{ 255, 185,   0 },{ 255, 186,   0 },{ 255, 186,   0 },{ 255, 187,   0 },
{ 255, 188,   0 },{ 255, 188,   0 },{ 255, 189,   0 },{ 255, 190,   0 },{ 255, 190,   0 },
{ 255, 191,   0 },{ 255, 192,   0 },{ 255, 192,   0 },{ 255, 193,   0 },{ 255, 194,   0 },
{ 255, 195,   0 },{ 255, 195,   0 },{ 255, 196,   0 },{ 255, 197,   0 },{ 255, 197,   0 },
{ 255, 198,   0 },{ 255, 199,   0 },{ 255, 199,   0 },{ 255, 200,   0 },{ 255, 201,   0 },
{ 255, 201,   0 },{ 255, 202,   0 },{ 255, 203,   0 },{ 255, 203,   0 },{ 255, 204,   0 },
{ 255, 205,   0 },{ 255, 205,   0 },{ 255, 206,   0 },{ 255, 207,   0 },{ 255, 207,   0 },
{ 255, 208,   0 },{ 255, 209,   0 },{ 255, 209,   0 },{ 255, 210,   0 },{ 255, 211,   0 },
{ 255, 211,   0 },{ 255, 212,   0 },{ 255, 213,   0 },{ 255, 213,   0 },{ 255, 214,   0 },
{ 255, 215,   0 },{ 255, 215,   0 },{ 255, 216,   0 },{ 255, 217,   0 },{ 255, 217,   0 },
{ 255, 218,   0 },{ 255, 219,   0 },{ 255, 219,   0 },{ 255, 220,   0 },{ 255, 221,   0 },
{ 255, 221,   0 },{ 255, 222,   0 },{ 255, 223,   0 },{ 255, 223,   0 },{ 255, 224,   0 },
{ 255, 225,   0 },{ 255, 225,   0 },{ 255, 226,   0 },{ 255, 227,   0 },{ 255, 227,   0 },
{ 255, 228,   0 },{ 255, 229,   0 },{ 255, 229,   0 },{ 255, 230,   0 },{ 255, 231,   0 },
{ 255, 231,   0 },{ 255, 232,   0 },{ 255, 233,   0 },{ 255, 233,   0 },{ 255, 234,   0 },
{ 255, 235,   0 },{ 255, 235,   0 },{ 255, 236,   0 },{ 255, 237,   0 },{ 255, 237,   0 },
{ 255, 238,   0 },{ 255, 239,   0 },{ 255, 239,   0 },{ 255, 240,   0 },{ 255, 241,   0 },
{ 255, 241,   0 },{ 255, 242,   0 },{ 255, 243,   0 },{ 255, 243,   0 },{ 255, 244,   0 },
{ 255, 245,   0 },{ 255, 245,   0 },{ 255, 246,   0 },{ 255, 247,   0 },{ 255, 247,   0 },
{ 255, 248,   0 },{ 255, 249,   0 },{ 255, 249,   0 },{ 255, 250,   0 },{ 255, 251,   0 },
{ 255, 251,   0 },{ 255, 252,   0 },{ 255, 253,   0 },{ 255, 253,   0 },{ 255, 254,   0 },
{ 255, 255,   0 },{ 255, 255,   1 },{ 255, 255,   2 },{ 255, 255,   3 },{ 255, 255,   4 },
{ 255, 255,   5 },{ 255, 255,   6 },{ 255, 255,   7 },{ 255, 255,   8 },{ 255, 255,   9 },
{ 255, 255,  10 },{ 255, 255,  11 },{ 255, 255,  12 },{ 255, 255,  13 },{ 255, 255,  14 },
{ 255, 255,  15 },{ 255, 255,  16 },{ 255, 255,  17 },{ 255, 255,  18 },{ 255, 255,  19 },
{ 255, 255,  20 },{ 255, 255,  21 },{ 255, 255,  22 },{ 255, 255,  23 },{ 255, 255,  24 },
{ 255, 255,  25 },{ 255, 255,  26 },{ 255, 255,  27 },{ 255, 255,  28 },{ 255, 255,  29 },
{ 255, 255,  30 },{ 255, 255,  31 },{ 255, 255,  32 },{ 255, 255,  33 },{ 255, 255,  34 },
{ 255, 255,  35 },{ 255, 255,  36 },{ 255, 255,  37 },{ 255, 255,  38 },{ 255, 255,  39 },
{ 255, 255,  40 },{ 255, 255,  41 },{ 255, 255,  42 },{ 255, 255,  43 },{ 255, 255,  44 },
{ 255, 255,  45 },{ 255, 255,  46 },{ 255, 255,  47 },{ 255, 255,  48 },{ 255, 255,  49 },
{ 255, 255,  50 },{ 255, 255,  51 },{ 255, 255,  52 },{ 255, 255,  53 },{ 255, 255,  54 },
{ 255, 255,  55 },{ 255, 255,  56 },{ 255, 255,  57 },{ 255, 255,  58 },{ 255, 255,  59 },
{ 255, 255,  60 },{ 255, 255,  61 },{ 255, 255,  62 },{ 255, 255,  63 },{ 255, 255,  64 },
{ 255, 255,  65 },{ 255, 255,  66 },{ 255, 255,  67 },{ 255, 255,  68 },{ 255, 255,  69 },
{ 255, 255,  70 },{ 255, 255,  71 },{ 255, 255,  72 },{ 255, 255,  73 },{ 255, 255,  74 },
{ 255, 255,  75 },{ 255, 255,  76 },{ 255, 255,  77 },{ 255, 255,  78 },{ 255, 255,  79 },
{ 255, 255,  80 },{ 255, 255,  81 },{ 255, 255,  82 },{ 255, 255,  83 },{ 255, 255,  84 },
{ 255, 255,  85 },{ 255, 255,  86 },{ 255, 255,  87 },{ 255, 255,  88 },{ 255, 255,  89 },
{ 255, 255,  90 },{ 255, 255,  91 },{ 255, 255,  92 },{ 255, 255,  93 },{ 255, 255,  94 },
{ 255, 255,  95 },{ 255, 255,  96 },{ 255, 255,  97 },{ 255, 255,  98 },{ 255, 255,  99 },
{ 255, 255, 100 },{ 255, 255, 101 },{ 255, 255, 102 },{ 255, 255, 103 },{ 255, 255, 104 },
{ 255, 255, 105 },{ 255, 255, 106 },{ 255, 255, 107 },{ 255, 255, 108 },{ 255, 255, 109 },
{ 255, 255, 110 },{ 255, 255, 111 },{ 255, 255, 112 },{ 255, 255, 113 },{ 255, 255, 114 },
{ 255, 255, 115 },{ 255, 255, 116 },{ 255, 255, 117 },{ 255, 255, 118 },{ 255, 255, 119 },
{ 255, 255, 120 },{ 255, 255, 121 },{ 255, 255, 122 },{ 255, 255, 123 },{ 255, 255, 124 },
{ 255, 255, 125 },{ 255, 255, 126 },{ 255, 255, 127 },{ 255, 255, 128 },{ 255, 255, 129 },
{ 255, 255, 130 },{ 255, 255, 131 },{ 255, 255, 132 },{ 255, 255, 133 },{ 255, 255, 134 },
{ 255, 255, 135 },{ 255, 255, 136 },{ 255, 255, 137 },{ 255, 255, 138 },{ 255, 255, 139 },
{ 255, 255, 140 },{ 255, 255, 141 },{ 255, 255, 142 },{ 255, 255, 143 },{ 255, 255, 144 },
{ 255, 255, 145 },{ 255, 255, 146 },{ 255, 255, 147 },{ 255, 255, 148 },{ 255, 255, 149 },
{ 255, 255, 150 },{ 255, 255, 151 },{ 255, 255, 152 },{ 255, 255, 153 },{ 255, 255, 154 },
{ 255, 255, 155 },{ 255, 255, 157 },{ 255, 255, 158 },{ 255, 255, 159 },{ 255, 255, 160 },
{ 255, 255, 161 },{ 255, 255, 162 },{ 255, 255, 163 },{ 255, 255, 164 },{ 255, 255, 165 },
{ 255, 255, 166 },{ 255, 255, 167 },{ 255, 255, 168 },{ 255, 255, 169 },{ 255, 255, 170 },
{ 255, 255, 171 },{ 255, 255, 172 },{ 255, 255, 173 },{ 255, 255, 174 },{ 255, 255, 175 },
{ 255, 255, 176 },{ 255, 255, 177 },{ 255, 255, 178 },{ 255, 255, 179 },{ 255, 255, 180 },
{ 255, 255, 181 },{ 255, 255, 182 },{ 255, 255, 183 },{ 255, 255, 184 },{ 255, 255, 185 },
{ 255, 255, 186 },{ 255, 255, 187 },{ 255, 255, 188 },{ 255, 255, 189 },{ 255, 255, 190 },
{ 255, 255, 191 },{ 255, 255, 192 },{ 255, 255, 193 },{ 255, 255, 194 },{ 255, 255, 195 },
{ 255, 255, 196 },{ 255, 255, 197 },{ 255, 255, 198 },{ 255, 255, 199 },{ 255, 255, 200 },
{ 255, 255, 201 },{ 255, 255, 202 },{ 255, 255, 203 },{ 255, 255, 204 },{ 255, 255, 205 },
{ 255, 255, 206 },{ 255, 255, 207 },{ 255, 255, 208 },{ 255, 255, 209 },{ 255, 255, 210 },
{ 255, 255, 211 },{ 255, 255, 212 },{ 255, 255, 213 },{ 255, 255, 214 },{ 255, 255, 215 },
{ 255, 255, 216 },{ 255, 255, 217 },{ 255, 255, 218 },{ 255, 255, 219 },{ 255, 255, 220 },
{ 255, 255, 221 },{ 255, 255, 222 },{ 255, 255, 223 },{ 255, 255, 224 },{ 255, 255, 225 },
{ 255, 255, 226 },{ 255, 255, 227 },{ 255, 255, 228 },{ 255, 255, 229 },{ 255, 255, 230 },
{ 255, 255, 231 },{ 255, 255, 232 },{ 255, 255, 233 },{ 255, 255, 234 },{ 255, 255, 235 },
{ 255, 255, 236 },{ 255, 255, 237 },{ 255, 255, 238 },{ 255, 255, 239 },{ 255, 255, 240 },
{ 255, 255, 241 },{ 255, 255, 242 },{ 255, 255, 243 },{ 255, 255, 244 },{ 255, 255, 245 },
{ 255, 255, 246 },{ 255, 255, 247 },{ 255, 255, 248 },{ 255, 255, 249 },{ 255, 255, 250 },
{ 255, 255, 251 },{ 255, 255, 252 },{ 255, 255, 253 },{ 255, 255, 254 },{ 255, 255, 255 }
};

const rgb_t hsv_colormap[1000] = {
    { 255,   0,   0 },{ 255,   2,   0 },{ 255,   3,   0 },{ 255,   5,   0 },{ 255,   6,   0 },
{ 255,   8,   0 },{ 255,   9,   0 },{ 255,  11,   0 },{ 255,  12,   0 },{ 255,  14,   0 },
{ 255,  15,   0 },{ 255,  17,   0 },{ 255,  18,   0 },{ 255,  20,   0 },{ 255,  21,   0 },
{ 255,  23,   0 },{ 255,  24,   0 },{ 255,  26,   0 },{ 255,  27,   0 },{ 255,  29,   0 },
{ 255,  30,   0 },{ 255,  32,   0 },{ 255,  33,   0 },{ 255,  35,   0 },{ 255,  36,   0 },
{ 255,  38,   0 },{ 255,  39,   0 },{ 255,  41,   0 },{ 255,  42,   0 },{ 255,  44,   0 },
{ 255,  45,   0 },{ 255,  47,   0 },{ 255,  48,   0 },{ 255,  50,   0 },{ 255,  51,   0 },
{ 255,  53,   0 },{ 255,  54,   0 },{ 255,  56,   0 },{ 255,  57,   0 },{ 255,  59,   0 },
{ 255,  60,   0 },{ 255,  62,   0 },{ 255,  63,   0 },{ 255,  65,   0 },{ 255,  66,   0 },
{ 255,  68,   0 },{ 255,  69,   0 },{ 255,  71,   0 },{ 255,  72,   0 },{ 255,  74,   0 },
{ 255,  75,   0 },{ 255,  77,   0 },{ 255,  78,   0 },{ 255,  80,   0 },{ 255,  81,   0 },
{ 255,  83,   0 },{ 255,  84,   0 },{ 255,  86,   0 },{ 255,  87,   0 },{ 255,  89,   0 },
{ 255,  90,   0 },{ 255,  92,   0 },{ 255,  93,   0 },{ 255,  95,   0 },{ 255,  96,   0 },
{ 255,  98,   0 },{ 255, 100,   0 },{ 255, 101,   0 },{ 255, 103,   0 },{ 255, 104,   0 },
{ 255, 106,   0 },{ 255, 107,   0 },{ 255, 109,   0 },{ 255, 110,   0 },{ 255, 112,   0 },
{ 255, 113,   0 },{ 255, 115,   0 },{ 255, 116,   0 },{ 255, 118,   0 },{ 255, 119,   0 },
{ 255, 121,   0 },{ 255, 122,   0 },{ 255, 124,   0 },{ 255, 125,   0 },{ 255, 127,   0 },
{ 255, 128,   0 },{ 255, 130,   0 },{ 255, 131,   0 },{ 255, 133,   0 },{ 255, 134,   0 },
{ 255, 136,   0 },{ 255, 137,   0 },{ 255, 139,   0 },{ 255, 140,   0 },{ 255, 142,   0 },
{ 255, 143,   0 },{ 255, 145,   0 },{ 255, 146,   0 },{ 255, 148,   0 },{ 255, 149,   0 },
{ 255, 151,   0 },{ 255, 152,   0 },{ 255, 154,   0 },{ 255, 155,   0 },{ 255, 157,   0 },
{ 255, 158,   0 },{ 255, 160,   0 },{ 255, 161,   0 },{ 255, 163,   0 },{ 255, 164,   0 },
{ 255, 166,   0 },{ 255, 167,   0 },{ 255, 169,   0 },{ 255, 170,   0 },{ 255, 172,   0 },
{ 255, 173,   0 },{ 255, 175,   0 },{ 255, 176,   0 },{ 255, 178,   0 },{ 255, 179,   0 },
{ 255, 181,   0 },{ 255, 182,   0 },{ 255, 184,   0 },{ 255, 185,   0 },{ 255, 187,   0 },
{ 255, 188,   0 },{ 255, 190,   0 },{ 255, 191,   0 },{ 255, 193,   0 },{ 255, 194,   0 },
{ 255, 196,   0 },{ 255, 197,   0 },{ 255, 199,   0 },{ 255, 201,   0 },{ 255, 202,   0 },
{ 255, 204,   0 },{ 255, 205,   0 },{ 255, 207,   0 },{ 255, 208,   0 },{ 255, 210,   0 },
{ 255, 211,   0 },{ 255, 213,   0 },{ 255, 214,   0 },{ 255, 216,   0 },{ 255, 217,   0 },
{ 255, 219,   0 },{ 255, 220,   0 },{ 255, 222,   0 },{ 255, 223,   0 },{ 255, 225,   0 },
{ 255, 226,   0 },{ 255, 228,   0 },{ 255, 229,   0 },{ 255, 231,   0 },{ 255, 232,   0 },
{ 255, 234,   0 },{ 255, 235,   0 },{ 255, 237,   0 },{ 255, 238,   0 },{ 255, 239,   0 },
{ 254, 240,   0 },{ 254, 242,   0 },{ 253, 243,   0 },{ 253, 244,   0 },{ 252, 245,   0 },
{ 252, 246,   0 },{ 251, 247,   0 },{ 251, 248,   0 },{ 250, 249,   0 },{ 250, 250,   0 },
{ 249, 251,   0 },{ 249, 252,   0 },{ 248, 253,   0 },{ 248, 254,   0 },{ 247, 255,   0 },
{ 246, 255,   0 },{ 245, 255,   0 },{ 243, 255,   0 },{ 242, 255,   0 },{ 240, 255,   0 },
{ 239, 255,   0 },{ 237, 255,   0 },{ 236, 255,   0 },{ 234, 255,   0 },{ 233, 255,   0 },
{ 231, 255,   0 },{ 230, 255,   0 },{ 228, 255,   0 },{ 227, 255,   0 },{ 225, 255,   0 },
{ 224, 255,   0 },{ 222, 255,   0 },{ 221, 255,   0 },{ 219, 255,   0 },{ 218, 255,   0 },
{ 216, 255,   0 },{ 215, 255,   0 },{ 213, 255,   0 },{ 211, 255,   0 },{ 210, 255,   0 },
{ 208, 255,   0 },{ 207, 255,   0 },{ 205, 255,   0 },{ 204, 255,   0 },{ 202, 255,   0 },
{ 201, 255,   0 },{ 199, 255,   0 },{ 198, 255,   0 },{ 196, 255,   0 },{ 195, 255,   0 },
{ 193, 255,   0 },{ 192, 255,   0 },{ 190, 255,   0 },{ 189, 255,   0 },{ 187, 255,   0 },
{ 186, 255,   0 },{ 184, 255,   0 },{ 183, 255,   0 },{ 181, 255,   0 },{ 180, 255,   0 },
{ 178, 255,   0 },{ 177, 255,   0 },{ 175, 255,   0 },{ 174, 255,   0 },{ 172, 255,   0 },
{ 171, 255,   0 },{ 169, 255,   0 },{ 168, 255,   0 },{ 166, 255,   0 },{ 165, 255,   0 },
{ 163, 255,   0 },{ 162, 255,   0 },{ 160, 255,   0 },{ 159, 255,   0 },{ 157, 255,   0 },
{ 156, 255,   0 },{ 154, 255,   0 },{ 153, 255,   0 },{ 151, 255,   0 },{ 150, 255,   0 },
{ 148, 255,   0 },{ 147, 255,   0 },{ 145, 255,   0 },{ 144, 255,   0 },{ 142, 255,   0 },
{ 141, 255,   0 },{ 139, 255,   0 },{ 138, 255,   0 },{ 136, 255,   0 },{ 135, 255,   0 },
{ 133, 255,   0 },{ 132, 255,   0 },{ 130, 255,   0 },{ 129, 255,   0 },{ 127, 255,   0 },
{ 126, 255,   0 },{ 124, 255,   0 },{ 123, 255,   0 },{ 121, 255,   0 },{ 120, 255,   0 },
{ 118, 255,   0 },{ 117, 255,   0 },{ 115, 255,   0 },{ 114, 255,   0 },{ 112, 255,   0 },
{ 110, 255,   0 },{ 109, 255,   0 },{ 107, 255,   0 },{ 106, 255,   0 },{ 104, 255,   0 },
{ 103, 255,   0 },{ 101, 255,   0 },{ 100, 255,   0 },{ 98, 255,   0 },{ 97, 255,   0 },
{ 95, 255,   0 },{ 94, 255,   0 },{ 92, 255,   0 },{ 91, 255,   0 },{ 89, 255,   0 },
{ 88, 255,   0 },{ 86, 255,   0 },{ 85, 255,   0 },{ 83, 255,   0 },{ 82, 255,   0 },
{ 80, 255,   0 },{ 79, 255,   0 },{ 77, 255,   0 },{ 76, 255,   0 },{ 74, 255,   0 },
{ 73, 255,   0 },{ 71, 255,   0 },{ 70, 255,   0 },{ 68, 255,   0 },{ 67, 255,   0 },
{ 65, 255,   0 },{ 64, 255,   0 },{ 62, 255,   0 },{ 61, 255,   0 },{ 59, 255,   0 },
{ 58, 255,   0 },{ 56, 255,   0 },{ 55, 255,   0 },{ 53, 255,   0 },{ 52, 255,   0 },
{ 50, 255,   0 },{ 49, 255,   0 },{ 47, 255,   0 },{ 46, 255,   0 },{ 44, 255,   0 },
{ 43, 255,   0 },{ 41, 255,   0 },{ 40, 255,   0 },{ 38, 255,   0 },{ 37, 255,   0 },
{ 35, 255,   0 },{ 34, 255,   0 },{ 32, 255,   0 },{ 31, 255,   0 },{ 29, 255,   0 },
{ 28, 255,   0 },{ 26, 255,   0 },{ 25, 255,   0 },{ 23, 255,   0 },{ 22, 255,   0 },
{ 20, 255,   0 },{ 19, 255,   0 },{ 17, 255,   0 },{ 16, 255,   0 },{ 14, 255,   0 },
{ 12, 255,   0 },{ 11, 255,   0 },{ 9, 255,   0 },{ 8, 255,   0 },{ 7, 255,   1 },
{ 7, 255,   2 },{ 6, 255,   3 },{ 6, 255,   4 },{ 5, 255,   5 },{ 5, 255,   6 },
{ 4, 255,   7 },{ 4, 255,   8 },{ 3, 255,   9 },{ 3, 255,  10 },{ 2, 255,  11 },
{ 2, 255,  12 },{ 1, 255,  13 },{ 1, 255,  14 },{ 0, 255,  15 },{ 0, 255,  16 },
{ 0, 255,  18 },{ 0, 255,  19 },{ 0, 255,  21 },{ 0, 255,  22 },{ 0, 255,  24 },
{ 0, 255,  25 },{ 0, 255,  27 },{ 0, 255,  28 },{ 0, 255,  30 },{ 0, 255,  31 },
{ 0, 255,  33 },{ 0, 255,  34 },{ 0, 255,  36 },{ 0, 255,  37 },{ 0, 255,  39 },
{ 0, 255,  40 },{ 0, 255,  42 },{ 0, 255,  43 },{ 0, 255,  45 },{ 0, 255,  46 },
{ 0, 255,  48 },{ 0, 255,  49 },{ 0, 255,  51 },{ 0, 255,  52 },{ 0, 255,  54 },
{ 0, 255,  55 },{ 0, 255,  57 },{ 0, 255,  58 },{ 0, 255,  60 },{ 0, 255,  61 },
{ 0, 255,  63 },{ 0, 255,  64 },{ 0, 255,  66 },{ 0, 255,  67 },{ 0, 255,  69 },
{ 0, 255,  70 },{ 0, 255,  72 },{ 0, 255,  73 },{ 0, 255,  75 },{ 0, 255,  76 },
{ 0, 255,  78 },{ 0, 255,  79 },{ 0, 255,  81 },{ 0, 255,  82 },{ 0, 255,  84 },
{ 0, 255,  86 },{ 0, 255,  87 },{ 0, 255,  89 },{ 0, 255,  90 },{ 0, 255,  92 },
{ 0, 255,  93 },{ 0, 255,  95 },{ 0, 255,  96 },{ 0, 255,  98 },{ 0, 255,  99 },
{ 0, 255, 101 },{ 0, 255, 102 },{ 0, 255, 104 },{ 0, 255, 105 },{ 0, 255, 107 },
{ 0, 255, 108 },{ 0, 255, 110 },{ 0, 255, 111 },{ 0, 255, 113 },{ 0, 255, 114 },
{ 0, 255, 116 },{ 0, 255, 117 },{ 0, 255, 119 },{ 0, 255, 120 },{ 0, 255, 122 },
{ 0, 255, 123 },{ 0, 255, 125 },{ 0, 255, 126 },{ 0, 255, 128 },{ 0, 255, 129 },
{ 0, 255, 131 },{ 0, 255, 132 },{ 0, 255, 134 },{ 0, 255, 135 },{ 0, 255, 137 },
{ 0, 255, 138 },{ 0, 255, 140 },{ 0, 255, 141 },{ 0, 255, 143 },{ 0, 255, 144 },
{ 0, 255, 146 },{ 0, 255, 147 },{ 0, 255, 149 },{ 0, 255, 150 },{ 0, 255, 152 },
{ 0, 255, 153 },{ 0, 255, 155 },{ 0, 255, 156 },{ 0, 255, 158 },{ 0, 255, 159 },
{ 0, 255, 161 },{ 0, 255, 162 },{ 0, 255, 164 },{ 0, 255, 165 },{ 0, 255, 167 },
{ 0, 255, 168 },{ 0, 255, 170 },{ 0, 255, 171 },{ 0, 255, 173 },{ 0, 255, 174 },
{ 0, 255, 176 },{ 0, 255, 177 },{ 0, 255, 179 },{ 0, 255, 180 },{ 0, 255, 182 },
{ 0, 255, 183 },{ 0, 255, 185 },{ 0, 255, 187 },{ 0, 255, 188 },{ 0, 255, 190 },
{ 0, 255, 191 },{ 0, 255, 193 },{ 0, 255, 194 },{ 0, 255, 196 },{ 0, 255, 197 },
{ 0, 255, 199 },{ 0, 255, 200 },{ 0, 255, 202 },{ 0, 255, 203 },{ 0, 255, 205 },
{ 0, 255, 206 },{ 0, 255, 208 },{ 0, 255, 209 },{ 0, 255, 211 },{ 0, 255, 212 },
{ 0, 255, 214 },{ 0, 255, 215 },{ 0, 255, 217 },{ 0, 255, 218 },{ 0, 255, 220 },
{ 0, 255, 221 },{ 0, 255, 223 },{ 0, 255, 224 },{ 0, 255, 226 },{ 0, 255, 227 },
{ 0, 255, 229 },{ 0, 255, 230 },{ 0, 255, 232 },{ 0, 255, 233 },{ 0, 255, 235 },
{ 0, 255, 236 },{ 0, 255, 238 },{ 0, 255, 239 },{ 0, 255, 241 },{ 0, 255, 242 },
{ 0, 255, 244 },{ 0, 255, 245 },{ 0, 255, 247 },{ 0, 255, 248 },{ 0, 255, 250 },
{ 0, 255, 251 },{ 0, 255, 253 },{ 0, 255, 254 },{ 0, 254, 255 },{ 0, 253, 255 },
{ 0, 251, 255 },{ 0, 250, 255 },{ 0, 248, 255 },{ 0, 247, 255 },{ 0, 245, 255 },
{ 0, 244, 255 },{ 0, 242, 255 },{ 0, 241, 255 },{ 0, 239, 255 },{ 0, 238, 255 },
{ 0, 236, 255 },{ 0, 235, 255 },{ 0, 233, 255 },{ 0, 232, 255 },{ 0, 230, 255 },
{ 0, 229, 255 },{ 0, 227, 255 },{ 0, 225, 255 },{ 0, 224, 255 },{ 0, 222, 255 },
{ 0, 221, 255 },{ 0, 219, 255 },{ 0, 218, 255 },{ 0, 216, 255 },{ 0, 215, 255 },
{ 0, 213, 255 },{ 0, 212, 255 },{ 0, 210, 255 },{ 0, 209, 255 },{ 0, 207, 255 },
{ 0, 206, 255 },{ 0, 204, 255 },{ 0, 203, 255 },{ 0, 201, 255 },{ 0, 200, 255 },
{ 0, 198, 255 },{ 0, 197, 255 },{ 0, 195, 255 },{ 0, 194, 255 },{ 0, 192, 255 },
{ 0, 191, 255 },{ 0, 189, 255 },{ 0, 188, 255 },{ 0, 186, 255 },{ 0, 185, 255 },
{ 0, 183, 255 },{ 0, 182, 255 },{ 0, 180, 255 },{ 0, 179, 255 },{ 0, 177, 255 },
{ 0, 176, 255 },{ 0, 174, 255 },{ 0, 173, 255 },{ 0, 171, 255 },{ 0, 170, 255 },
{ 0, 168, 255 },{ 0, 167, 255 },{ 0, 165, 255 },{ 0, 164, 255 },{ 0, 162, 255 },
{ 0, 161, 255 },{ 0, 159, 255 },{ 0, 158, 255 },{ 0, 156, 255 },{ 0, 155, 255 },
{ 0, 153, 255 },{ 0, 152, 255 },{ 0, 150, 255 },{ 0, 149, 255 },{ 0, 147, 255 },
{ 0, 146, 255 },{ 0, 144, 255 },{ 0, 143, 255 },{ 0, 141, 255 },{ 0, 140, 255 },
{ 0, 138, 255 },{ 0, 137, 255 },{ 0, 135, 255 },{ 0, 134, 255 },{ 0, 132, 255 },
{ 0, 131, 255 },{ 0, 129, 255 },{ 0, 128, 255 },{ 0, 126, 255 },{ 0, 124, 255 },
{ 0, 123, 255 },{ 0, 121, 255 },{ 0, 120, 255 },{ 0, 118, 255 },{ 0, 117, 255 },
{ 0, 115, 255 },{ 0, 114, 255 },{ 0, 112, 255 },{ 0, 111, 255 },{ 0, 109, 255 },
{ 0, 108, 255 },{ 0, 106, 255 },{ 0, 105, 255 },{ 0, 103, 255 },{ 0, 102, 255 },
{ 0, 100, 255 },{ 0,  99, 255 },{ 0,  97, 255 },{ 0,  96, 255 },{ 0,  94, 255 },
{ 0,  93, 255 },{ 0,  91, 255 },{ 0,  90, 255 },{ 0,  88, 255 },{ 0,  87, 255 },
{ 0,  85, 255 },{ 0,  84, 255 },{ 0,  82, 255 },{ 0,  81, 255 },{ 0,  79, 255 },
{ 0,  78, 255 },{ 0,  76, 255 },{ 0,  75, 255 },{ 0,  73, 255 },{ 0,  72, 255 },
{ 0,  70, 255 },{ 0,  69, 255 },{ 0,  67, 255 },{ 0,  66, 255 },{ 0,  64, 255 },
{ 0,  63, 255 },{ 0,  61, 255 },{ 0,  60, 255 },{ 0,  58, 255 },{ 0,  57, 255 },
{ 0,  55, 255 },{ 0,  54, 255 },{ 0,  52, 255 },{ 0,  51, 255 },{ 0,  49, 255 },
{ 0,  48, 255 },{ 0,  46, 255 },{ 0,  45, 255 },{ 0,  43, 255 },{ 0,  42, 255 },
{ 0,  40, 255 },{ 0,  39, 255 },{ 0,  37, 255 },{ 0,  36, 255 },{ 0,  34, 255 },
{ 0,  33, 255 },{ 0,  31, 255 },{ 0,  30, 255 },{ 0,  28, 255 },{ 0,  26, 255 },
{ 0,  25, 255 },{ 0,  23, 255 },{ 0,  22, 255 },{ 0,  20, 255 },{ 0,  19, 255 },
{ 0,  17, 255 },{ 0,  16, 255 },{ 1,  15, 255 },{ 1,  14, 255 },{ 2,  13, 255 },
{ 2,  12, 255 },{ 3,  11, 255 },{ 3,  10, 255 },{ 4,   9, 255 },{ 4,   8, 255 },
{ 5,   7, 255 },{ 5,   6, 255 },{ 6,   5, 255 },{ 6,   4, 255 },{ 7,   3, 255 },
{ 7,   2, 255 },{ 8,   1, 255 },{ 8,   0, 255 },{ 10,   0, 255 },{ 11,   0, 255 },
{ 13,   0, 255 },{ 14,   0, 255 },{ 16,   0, 255 },{ 17,   0, 255 },{ 19,   0, 255 },
{ 20,   0, 255 },{ 22,   0, 255 },{ 23,   0, 255 },{ 25,   0, 255 },{ 26,   0, 255 },
{ 28,   0, 255 },{ 29,   0, 255 },{ 31,   0, 255 },{ 32,   0, 255 },{ 34,   0, 255 },
{ 35,   0, 255 },{ 37,   0, 255 },{ 38,   0, 255 },{ 40,   0, 255 },{ 41,   0, 255 },
{ 43,   0, 255 },{ 44,   0, 255 },{ 46,   0, 255 },{ 47,   0, 255 },{ 49,   0, 255 },
{ 50,   0, 255 },{ 52,   0, 255 },{ 53,   0, 255 },{ 55,   0, 255 },{ 56,   0, 255 },
{ 58,   0, 255 },{ 59,   0, 255 },{ 61,   0, 255 },{ 62,   0, 255 },{ 64,   0, 255 },
{ 65,   0, 255 },{ 67,   0, 255 },{ 68,   0, 255 },{ 70,   0, 255 },{ 72,   0, 255 },
{ 73,   0, 255 },{ 75,   0, 255 },{ 76,   0, 255 },{ 78,   0, 255 },{ 79,   0, 255 },
{ 81,   0, 255 },{ 82,   0, 255 },{ 84,   0, 255 },{ 85,   0, 255 },{ 87,   0, 255 },
{ 88,   0, 255 },{ 90,   0, 255 },{ 91,   0, 255 },{ 93,   0, 255 },{ 94,   0, 255 },
{ 96,   0, 255 },{ 97,   0, 255 },{ 99,   0, 255 },{ 100,   0, 255 },{ 102,   0, 255 },
{ 103,   0, 255 },{ 105,   0, 255 },{ 106,   0, 255 },{ 108,   0, 255 },{ 109,   0, 255 },
{ 111,   0, 255 },{ 112,   0, 255 },{ 114,   0, 255 },{ 115,   0, 255 },{ 117,   0, 255 },
{ 118,   0, 255 },{ 120,   0, 255 },{ 121,   0, 255 },{ 123,   0, 255 },{ 124,   0, 255 },
{ 126,   0, 255 },{ 127,   0, 255 },{ 129,   0, 255 },{ 130,   0, 255 },{ 132,   0, 255 },
{ 133,   0, 255 },{ 135,   0, 255 },{ 136,   0, 255 },{ 138,   0, 255 },{ 139,   0, 255 },
{ 141,   0, 255 },{ 142,   0, 255 },{ 144,   0, 255 },{ 145,   0, 255 },{ 147,   0, 255 },
{ 148,   0, 255 },{ 150,   0, 255 },{ 151,   0, 255 },{ 153,   0, 255 },{ 154,   0, 255 },
{ 156,   0, 255 },{ 157,   0, 255 },{ 159,   0, 255 },{ 160,   0, 255 },{ 162,   0, 255 },
{ 163,   0, 255 },{ 165,   0, 255 },{ 166,   0, 255 },{ 168,   0, 255 },{ 169,   0, 255 },
{ 171,   0, 255 },{ 173,   0, 255 },{ 174,   0, 255 },{ 176,   0, 255 },{ 177,   0, 255 },
{ 179,   0, 255 },{ 180,   0, 255 },{ 182,   0, 255 },{ 183,   0, 255 },{ 185,   0, 255 },
{ 186,   0, 255 },{ 188,   0, 255 },{ 189,   0, 255 },{ 191,   0, 255 },{ 192,   0, 255 },
{ 194,   0, 255 },{ 195,   0, 255 },{ 197,   0, 255 },{ 198,   0, 255 },{ 200,   0, 255 },
{ 201,   0, 255 },{ 203,   0, 255 },{ 204,   0, 255 },{ 206,   0, 255 },{ 207,   0, 255 },
{ 209,   0, 255 },{ 210,   0, 255 },{ 212,   0, 255 },{ 213,   0, 255 },{ 215,   0, 255 },
{ 216,   0, 255 },{ 218,   0, 255 },{ 219,   0, 255 },{ 221,   0, 255 },{ 222,   0, 255 },
{ 224,   0, 255 },{ 225,   0, 255 },{ 227,   0, 255 },{ 228,   0, 255 },{ 230,   0, 255 },
{ 231,   0, 255 },{ 233,   0, 255 },{ 234,   0, 255 },{ 236,   0, 255 },{ 237,   0, 255 },
{ 239,   0, 255 },{ 240,   0, 255 },{ 242,   0, 255 },{ 243,   0, 255 },{ 245,   0, 255 },
{ 246,   0, 255 },{ 247,   0, 254 },{ 248,   0, 253 },{ 248,   0, 252 },{ 249,   0, 251 },
{ 249,   0, 250 },{ 250,   0, 249 },{ 250,   0, 248 },{ 251,   0, 247 },{ 251,   0, 246 },
{ 252,   0, 245 },{ 252,   0, 244 },{ 253,   0, 243 },{ 253,   0, 242 },{ 254,   0, 241 },
{ 254,   0, 240 },{ 255,   0, 239 },{ 255,   0, 238 },{ 255,   0, 236 },{ 255,   0, 235 },
{ 255,   0, 233 },{ 255,   0, 232 },{ 255,   0, 230 },{ 255,   0, 229 },{ 255,   0, 227 },
{ 255,   0, 226 },{ 255,   0, 224 },{ 255,   0, 223 },{ 255,   0, 221 },{ 255,   0, 220 },
{ 255,   0, 218 },{ 255,   0, 217 },{ 255,   0, 215 },{ 255,   0, 214 },{ 255,   0, 212 },
{ 255,   0, 211 },{ 255,   0, 209 },{ 255,   0, 208 },{ 255,   0, 206 },{ 255,   0, 205 },
{ 255,   0, 203 },{ 255,   0, 202 },{ 255,   0, 200 },{ 255,   0, 199 },{ 255,   0, 197 },
{ 255,   0, 196 },{ 255,   0, 194 },{ 255,   0, 193 },{ 255,   0, 191 },{ 255,   0, 190 },
{ 255,   0, 188 },{ 255,   0, 187 },{ 255,   0, 185 },{ 255,   0, 184 },{ 255,   0, 182 },
{ 255,   0, 181 },{ 255,   0, 179 },{ 255,   0, 178 },{ 255,   0, 176 },{ 255,   0, 175 },
{ 255,   0, 173 },{ 255,   0, 172 },{ 255,   0, 170 },{ 255,   0, 169 },{ 255,   0, 167 },
{ 255,   0, 166 },{ 255,   0, 164 },{ 255,   0, 163 },{ 255,   0, 161 },{ 255,   0, 160 },
{ 255,   0, 158 },{ 255,   0, 157 },{ 255,   0, 155 },{ 255,   0, 154 },{ 255,   0, 152 },
{ 255,   0, 151 },{ 255,   0, 149 },{ 255,   0, 148 },{ 255,   0, 146 },{ 255,   0, 145 },
{ 255,   0, 143 },{ 255,   0, 141 },{ 255,   0, 140 },{ 255,   0, 138 },{ 255,   0, 137 },
{ 255,   0, 135 },{ 255,   0, 134 },{ 255,   0, 132 },{ 255,   0, 131 },{ 255,   0, 129 },
{ 255,   0, 128 },{ 255,   0, 126 },{ 255,   0, 125 },{ 255,   0, 123 },{ 255,   0, 122 },
{ 255,   0, 120 },{ 255,   0, 119 },{ 255,   0, 117 },{ 255,   0, 116 },{ 255,   0, 114 },
{ 255,   0, 113 },{ 255,   0, 111 },{ 255,   0, 110 },{ 255,   0, 108 },{ 255,   0, 107 },
{ 255,   0, 105 },{ 255,   0, 104 },{ 255,   0, 102 },{ 255,   0, 101 },{ 255,   0,  99 },
{ 255,   0,  98 },{ 255,   0,  96 },{ 255,   0,  95 },{ 255,   0,  93 },{ 255,   0,  92 },
{ 255,   0,  90 },{ 255,   0,  89 },{ 255,   0,  87 },{ 255,   0,  86 },{ 255,   0,  84 },
{ 255,   0,  83 },{ 255,   0,  81 },{ 255,   0,  80 },{ 255,   0,  78 },{ 255,   0,  77 },
{ 255,   0,  75 },{ 255,   0,  74 },{ 255,   0,  72 },{ 255,   0,  71 },{ 255,   0,  69 },
{ 255,   0,  68 },{ 255,   0,  66 },{ 255,   0,  65 },{ 255,   0,  63 },{ 255,   0,  62 },
{ 255,   0,  60 },{ 255,   0,  59 },{ 255,   0,  57 },{ 255,   0,  56 },{ 255,   0,  54 },
{ 255,   0,  53 },{ 255,   0,  51 },{ 255,   0,  50 },{ 255,   0,  48 },{ 255,   0,  47 },
{ 255,   0,  45 },{ 255,   0,  44 },{ 255,   0,  42 },{ 255,   0,  40 },{ 255,   0,  39 },
{ 255,   0,  37 },{ 255,   0,  36 },{ 255,   0,  34 },{ 255,   0,  33 },{ 255,   0,  31 },
{ 255,   0,  30 },{ 255,   0,  28 },{ 255,   0,  27 },{ 255,   0,  25 },{ 255,   0,  24 }
};

const rgb_t jet_colormap[1000] = {
    { 29,   0, 102 },{ 23,   0, 107 },{ 17,   0, 112 },{ 12,   0, 117 },{ 6,   0, 122 },
{ 0,   0, 127 },{ 0,   0, 128 },{ 0,   0, 129 },{ 0,   0, 129 },{ 0,   0, 130 },
{ 0,   0, 131 },{ 0,   0, 132 },{ 0,   0, 133 },{ 0,   0, 133 },{ 0,   0, 134 },
{ 0,   0, 135 },{ 0,   0, 136 },{ 0,   0, 137 },{ 0,   0, 138 },{ 0,   0, 140 },
{ 0,   0, 141 },{ 0,   0, 142 },{ 0,   0, 143 },{ 0,   0, 145 },{ 0,   0, 146 },
{ 0,   0, 147 },{ 0,   0, 148 },{ 0,   0, 150 },{ 0,   0, 151 },{ 0,   0, 152 },
{ 0,   0, 153 },{ 0,   0, 154 },{ 0,   0, 156 },{ 0,   0, 157 },{ 0,   0, 158 },
{ 0,   0, 159 },{ 0,   0, 160 },{ 0,   0, 161 },{ 0,   0, 163 },{ 0,   0, 164 },
{ 0,   0, 165 },{ 0,   0, 166 },{ 0,   0, 168 },{ 0,   0, 169 },{ 0,   0, 170 },
{ 0,   0, 171 },{ 0,   0, 173 },{ 0,   0, 174 },{ 0,   0, 175 },{ 0,   0, 176 },
{ 0,   0, 178 },{ 0,   0, 179 },{ 0,   0, 180 },{ 0,   0, 181 },{ 0,   0, 183 },
{ 0,   0, 184 },{ 0,   0, 185 },{ 0,   0, 186 },{ 0,   0, 188 },{ 0,   0, 189 },
{ 0,   0, 190 },{ 0,   0, 191 },{ 0,   0, 193 },{ 0,   0, 194 },{ 0,   0, 195 },
{ 0,   0, 196 },{ 0,   0, 197 },{ 0,   0, 198 },{ 0,   0, 200 },{ 0,   0, 201 },
{ 0,   0, 202 },{ 0,   0, 203 },{ 0,   0, 204 },{ 0,   0, 206 },{ 0,   0, 207 },
{ 0,   0, 208 },{ 0,   0, 209 },{ 0,   0, 211 },{ 0,   0, 212 },{ 0,   0, 213 },
{ 0,   0, 214 },{ 0,   0, 216 },{ 0,   0, 217 },{ 0,   0, 218 },{ 0,   0, 219 },
{ 0,   0, 221 },{ 0,   0, 222 },{ 0,   0, 223 },{ 0,   0, 225 },{ 0,   0, 226 },
{ 0,   0, 227 },{ 0,   0, 228 },{ 0,   0, 230 },{ 0,   0, 231 },{ 0,   0, 232 },
{ 0,   0, 233 },{ 0,   0, 234 },{ 0,   0, 234 },{ 0,   0, 235 },{ 0,   0, 236 },
{ 0,   0, 237 },{ 0,   0, 238 },{ 0,   0, 239 },{ 0,   0, 239 },{ 0,   0, 240 },
{ 0,   0, 241 },{ 0,   0, 242 },{ 0,   0, 243 },{ 0,   0, 244 },{ 0,   0, 246 },
{ 0,   0, 247 },{ 0,   0, 248 },{ 0,   0, 249 },{ 0,   0, 250 },{ 0,   0, 251 },
{ 0,   0, 253 },{ 0,   0, 254 },{ 0,   0, 254 },{ 0,   0, 254 },{ 0,   0, 254 },
{ 0,   0, 254 },{ 0,   0, 254 },{ 0,   0, 255 },{ 0,   0, 255 },{ 0,   0, 255 },
{ 0,   0, 255 },{ 0,   0, 255 },{ 0,   0, 255 },{ 0,   1, 255 },{ 0,   1, 255 },
{ 0,   2, 255 },{ 0,   3, 255 },{ 0,   3, 255 },{ 0,   4, 255 },{ 0,   5, 255 },
{ 0,   6, 255 },{ 0,   6, 255 },{ 0,   7, 255 },{ 0,   8, 255 },{ 0,   9, 255 },
{ 0,  10, 255 },{ 0,  11, 255 },{ 0,  12, 255 },{ 0,  13, 255 },{ 0,  14, 255 },
{ 0,  15, 255 },{ 0,  16, 255 },{ 0,  17, 255 },{ 0,  18, 255 },{ 0,  19, 255 },
{ 0,  21, 255 },{ 0,  22, 255 },{ 0,  23, 255 },{ 0,  24, 255 },{ 0,  25, 255 },
{ 0,  26, 255 },{ 0,  27, 255 },{ 0,  28, 255 },{ 0,  29, 255 },{ 0,  30, 255 },
{ 0,  31, 255 },{ 0,  32, 255 },{ 0,  34, 255 },{ 0,  35, 255 },{ 0,  36, 255 },
{ 0,  37, 255 },{ 0,  38, 255 },{ 0,  39, 255 },{ 0,  40, 255 },{ 0,  41, 255 },
{ 0,  42, 255 },{ 0,  43, 255 },{ 0,  44, 255 },{ 0,  45, 255 },{ 0,  46, 255 },
{ 0,  48, 255 },{ 0,  49, 255 },{ 0,  50, 255 },{ 0,  51, 255 },{ 0,  52, 255 },
{ 0,  53, 255 },{ 0,  54, 255 },{ 0,  55, 255 },{ 0,  56, 255 },{ 0,  57, 255 },
{ 0,  58, 255 },{ 0,  58, 255 },{ 0,  59, 255 },{ 0,  60, 255 },{ 0,  60, 255 },
{ 0,  61, 255 },{ 0,  62, 255 },{ 0,  63, 255 },{ 0,  63, 255 },{ 0,  64, 255 },
{ 0,  65, 255 },{ 0,  66, 255 },{ 0,  67, 255 },{ 0,  68, 255 },{ 0,  69, 255 },
{ 0,  71, 255 },{ 0,  72, 255 },{ 0,  73, 255 },{ 0,  74, 255 },{ 0,  75, 255 },
{ 0,  76, 255 },{ 0,  77, 255 },{ 0,  78, 255 },{ 0,  79, 255 },{ 0,  80, 255 },
{ 0,  81, 255 },{ 0,  82, 255 },{ 0,  84, 255 },{ 0,  85, 255 },{ 0,  86, 255 },
{ 0,  87, 255 },{ 0,  88, 255 },{ 0,  89, 255 },{ 0,  90, 255 },{ 0,  91, 255 },
{ 0,  92, 255 },{ 0,  93, 255 },{ 0,  94, 255 },{ 0,  95, 255 },{ 0,  96, 255 },
{ 0,  98, 255 },{ 0,  99, 255 },{ 0, 100, 255 },{ 0, 101, 255 },{ 0, 102, 255 },
{ 0, 103, 255 },{ 0, 104, 255 },{ 0, 105, 255 },{ 0, 106, 255 },{ 0, 107, 255 },
{ 0, 108, 255 },{ 0, 109, 255 },{ 0, 111, 255 },{ 0, 112, 255 },{ 0, 113, 255 },
{ 0, 114, 255 },{ 0, 115, 255 },{ 0, 116, 255 },{ 0, 117, 255 },{ 0, 118, 255 },
{ 0, 119, 255 },{ 0, 120, 255 },{ 0, 121, 255 },{ 0, 122, 255 },{ 0, 123, 255 },
{ 0, 125, 255 },{ 0, 126, 255 },{ 0, 127, 255 },{ 0, 128, 255 },{ 0, 129, 255 },
{ 0, 130, 255 },{ 0, 131, 255 },{ 0, 132, 255 },{ 0, 133, 255 },{ 0, 134, 255 },
{ 0, 135, 255 },{ 0, 136, 255 },{ 0, 138, 255 },{ 0, 139, 255 },{ 0, 140, 255 },
{ 0, 141, 255 },{ 0, 142, 255 },{ 0, 143, 255 },{ 0, 144, 255 },{ 0, 145, 255 },
{ 0, 146, 255 },{ 0, 147, 255 },{ 0, 148, 255 },{ 0, 149, 255 },{ 0, 150, 255 },
{ 0, 150, 255 },{ 0, 151, 255 },{ 0, 152, 255 },{ 0, 153, 255 },{ 0, 153, 255 },
{ 0, 154, 255 },{ 0, 155, 255 },{ 0, 155, 255 },{ 0, 156, 255 },{ 0, 157, 255 },
{ 0, 158, 255 },{ 0, 159, 255 },{ 0, 161, 255 },{ 0, 162, 255 },{ 0, 163, 255 },
{ 0, 164, 255 },{ 0, 165, 255 },{ 0, 166, 255 },{ 0, 167, 255 },{ 0, 168, 255 },
{ 0, 169, 255 },{ 0, 170, 255 },{ 0, 171, 255 },{ 0, 172, 255 },{ 0, 173, 255 },
{ 0, 175, 255 },{ 0, 176, 255 },{ 0, 177, 255 },{ 0, 178, 255 },{ 0, 179, 255 },
{ 0, 180, 255 },{ 0, 181, 255 },{ 0, 182, 255 },{ 0, 183, 255 },{ 0, 184, 255 },
{ 0, 185, 255 },{ 0, 186, 255 },{ 0, 188, 255 },{ 0, 189, 255 },{ 0, 190, 255 },
{ 0, 191, 255 },{ 0, 192, 255 },{ 0, 193, 255 },{ 0, 194, 255 },{ 0, 195, 255 },
{ 0, 196, 255 },{ 0, 197, 255 },{ 0, 198, 255 },{ 0, 199, 255 },{ 0, 200, 255 },
{ 0, 202, 255 },{ 0, 203, 255 },{ 0, 204, 255 },{ 0, 205, 255 },{ 0, 206, 255 },
{ 0, 207, 255 },{ 0, 208, 255 },{ 0, 209, 255 },{ 0, 210, 255 },{ 0, 211, 255 },
{ 0, 212, 255 },{ 0, 213, 255 },{ 0, 215, 255 },{ 0, 216, 255 },{ 0, 217, 255 },
{ 0, 218, 254 },{ 0, 219, 253 },{ 0, 220, 252 },{ 0, 221, 252 },{ 0, 222, 251 },
{ 0, 223, 250 },{ 0, 224, 250 },{ 0, 225, 249 },{ 0, 226, 248 },{ 0, 227, 247 },
{ 0, 229, 247 },{ 1, 230, 246 },{ 2, 231, 245 },{ 3, 232, 244 },{ 3, 233, 243 },
{ 4, 234, 242 },{ 5, 235, 241 },{ 5, 236, 240 },{ 6, 237, 239 },{ 7, 238, 238 },
{ 8, 239, 238 },{ 8, 240, 237 },{ 9, 241, 236 },{ 10, 242, 236 },{ 10, 242, 235 },
{ 11, 243, 235 },{ 11, 244, 234 },{ 12, 245, 234 },{ 13, 245, 233 },{ 13, 246, 232 },
{ 14, 247, 232 },{ 15, 247, 231 },{ 15, 248, 231 },{ 16, 249, 230 },{ 17, 249, 229 },
{ 18, 250, 228 },{ 18, 251, 227 },{ 19, 251, 226 },{ 20, 252, 225 },{ 21, 253, 224 },
{ 22, 253, 224 },{ 23, 254, 223 },{ 23, 254, 222 },{ 24, 255, 221 },{ 25, 255, 220 },
{ 26, 255, 219 },{ 27, 255, 218 },{ 28, 255, 218 },{ 29, 255, 217 },{ 30, 255, 216 },
{ 30, 255, 215 },{ 31, 255, 214 },{ 32, 255, 214 },{ 33, 255, 213 },{ 34, 255, 212 },
{ 35, 255, 211 },{ 36, 255, 210 },{ 37, 255, 209 },{ 38, 255, 208 },{ 39, 255, 207 },
{ 39, 255, 207 },{ 40, 255, 206 },{ 41, 255, 205 },{ 42, 255, 204 },{ 43, 255, 203 },
{ 44, 255, 202 },{ 45, 255, 201 },{ 46, 255, 200 },{ 47, 255, 199 },{ 48, 255, 198 },
{ 48, 255, 198 },{ 49, 255, 197 },{ 50, 255, 196 },{ 51, 255, 195 },{ 52, 255, 194 },
{ 53, 255, 193 },{ 54, 255, 192 },{ 55, 255, 191 },{ 55, 255, 191 },{ 56, 255, 190 },
{ 57, 255, 189 },{ 58, 255, 188 },{ 59, 255, 187 },{ 60, 255, 186 },{ 60, 255, 186 },
{ 61, 255, 185 },{ 62, 255, 184 },{ 63, 255, 183 },{ 64, 255, 182 },{ 65, 255, 181 },
{ 65, 255, 181 },{ 66, 255, 180 },{ 67, 255, 179 },{ 68, 255, 178 },{ 69, 255, 177 },
{ 70, 255, 176 },{ 71, 255, 175 },{ 72, 255, 174 },{ 73, 255, 173 },{ 74, 255, 172 },
{ 74, 255, 172 },{ 75, 255, 171 },{ 76, 255, 170 },{ 77, 255, 169 },{ 78, 255, 168 },
{ 79, 255, 167 },{ 80, 255, 166 },{ 81, 255, 165 },{ 82, 255, 164 },{ 83, 255, 163 },
{ 83, 255, 163 },{ 84, 255, 162 },{ 84, 255, 162 },{ 85, 255, 161 },{ 85, 255, 161 },
{ 86, 255, 160 },{ 87, 255, 159 },{ 87, 255, 159 },{ 88, 255, 158 },{ 88, 255, 158 },
{ 89, 255, 157 },{ 89, 255, 157 },{ 90, 255, 156 },{ 91, 255, 155 },{ 92, 255, 154 },
{ 93, 255, 153 },{ 94, 255, 152 },{ 95, 255, 151 },{ 96, 255, 150 },{ 97, 255, 149 },
{ 97, 255, 149 },{ 98, 255, 148 },{ 99, 255, 147 },{ 100, 255, 146 },{ 101, 255, 145 },
{ 102, 255, 144 },{ 102, 255, 143 },{ 103, 255, 142 },{ 104, 255, 141 },{ 105, 255, 140 },
{ 106, 255, 140 },{ 107, 255, 139 },{ 107, 255, 138 },{ 108, 255, 137 },{ 109, 255, 136 },
{ 110, 255, 135 },{ 111, 255, 134 },{ 112, 255, 134 },{ 113, 255, 133 },{ 114, 255, 132 },
{ 114, 255, 131 },{ 115, 255, 130 },{ 116, 255, 130 },{ 117, 255, 129 },{ 118, 255, 128 },
{ 119, 255, 127 },{ 120, 255, 126 },{ 121, 255, 125 },{ 122, 255, 124 },{ 123, 255, 123 },
{ 123, 255, 123 },{ 124, 255, 122 },{ 125, 255, 121 },{ 126, 255, 120 },{ 127, 255, 119 },
{ 128, 255, 118 },{ 129, 255, 117 },{ 130, 255, 116 },{ 130, 255, 115 },{ 131, 255, 114 },
{ 132, 255, 114 },{ 133, 255, 113 },{ 134, 255, 112 },{ 134, 255, 111 },{ 135, 255, 110 },
{ 136, 255, 109 },{ 137, 255, 108 },{ 138, 255, 107 },{ 139, 255, 107 },{ 140, 255, 106 },
{ 140, 255, 105 },{ 141, 255, 104 },{ 142, 255, 103 },{ 143, 255, 102 },{ 144, 255, 102 },
{ 145, 255, 101 },{ 146, 255, 100 },{ 147, 255,  99 },{ 148, 255,  98 },{ 149, 255,  97 },
{ 149, 255,  97 },{ 150, 255,  96 },{ 151, 255,  95 },{ 152, 255,  94 },{ 153, 255,  93 },
{ 154, 255,  92 },{ 155, 255,  91 },{ 156, 255,  90 },{ 157, 255,  89 },{ 157, 255,  89 },
{ 158, 255,  88 },{ 158, 255,  88 },{ 159, 255,  87 },{ 159, 255,  87 },{ 160, 255,  86 },
{ 161, 255,  85 },{ 161, 255,  85 },{ 162, 255,  84 },{ 162, 255,  84 },{ 163, 255,  83 },
{ 163, 255,  83 },{ 164, 255,  82 },{ 165, 255,  81 },{ 166, 255,  80 },{ 167, 255,  79 },
{ 168, 255,  78 },{ 169, 255,  77 },{ 170, 255,  76 },{ 171, 255,  75 },{ 172, 255,  74 },
{ 172, 255,  74 },{ 173, 255,  73 },{ 174, 255,  72 },{ 175, 255,  71 },{ 176, 255,  70 },
{ 177, 255,  69 },{ 178, 255,  68 },{ 179, 255,  67 },{ 180, 255,  66 },{ 181, 255,  65 },
{ 181, 255,  65 },{ 182, 255,  64 },{ 183, 255,  63 },{ 184, 255,  62 },{ 185, 255,  61 },
{ 186, 255,  60 },{ 186, 255,  60 },{ 187, 255,  59 },{ 188, 255,  58 },{ 189, 255,  57 },
{ 190, 255,  56 },{ 191, 255,  55 },{ 191, 255,  55 },{ 192, 255,  54 },{ 193, 255,  53 },
{ 194, 255,  52 },{ 195, 255,  51 },{ 196, 255,  50 },{ 197, 255,  49 },{ 198, 255,  48 },
{ 198, 255,  48 },{ 199, 255,  47 },{ 200, 255,  46 },{ 201, 255,  45 },{ 202, 255,  44 },
{ 203, 255,  43 },{ 204, 255,  42 },{ 205, 255,  41 },{ 206, 255,  40 },{ 207, 255,  39 },
{ 207, 255,  39 },{ 208, 255,  38 },{ 209, 255,  37 },{ 210, 255,  36 },{ 211, 255,  35 },
{ 212, 255,  34 },{ 213, 255,  33 },{ 214, 255,  32 },{ 214, 255,  31 },{ 215, 255,  30 },
{ 216, 255,  30 },{ 217, 255,  29 },{ 218, 255,  28 },{ 218, 255,  27 },{ 219, 255,  26 },
{ 220, 255,  25 },{ 221, 255,  24 },{ 222, 255,  23 },{ 223, 255,  23 },{ 224, 255,  22 },
{ 224, 255,  21 },{ 225, 255,  20 },{ 226, 255,  19 },{ 227, 255,  18 },{ 228, 255,  18 },
{ 229, 255,  17 },{ 230, 255,  16 },{ 231, 255,  15 },{ 231, 255,  15 },{ 232, 255,  14 },
{ 232, 255,  13 },{ 233, 255,  13 },{ 234, 255,  12 },{ 234, 255,  11 },{ 235, 255,  11 },
{ 235, 255,  10 },{ 236, 255,  10 },{ 236, 255,   9 },{ 237, 255,   8 },{ 238, 254,   8 },
{ 238, 253,   7 },{ 239, 252,   6 },{ 240, 251,   5 },{ 241, 250,   5 },{ 242, 249,   4 },
{ 243, 248,   3 },{ 244, 247,   3 },{ 245, 246,   2 },{ 246, 246,   1 },{ 247, 245,   0 },
{ 247, 243,   0 },{ 248, 242,   0 },{ 249, 242,   0 },{ 250, 241,   0 },{ 250, 240,   0 },
{ 251, 239,   0 },{ 252, 238,   0 },{ 252, 237,   0 },{ 253, 236,   0 },{ 254, 235,   0 },
{ 255, 234,   0 },{ 255, 233,   0 },{ 255, 232,   0 },{ 255, 231,   0 },{ 255, 230,   0 },
{ 255, 229,   0 },{ 255, 228,   0 },{ 255, 227,   0 },{ 255, 226,   0 },{ 255, 225,   0 },
{ 255, 224,   0 },{ 255, 223,   0 },{ 255, 222,   0 },{ 255, 221,   0 },{ 255, 220,   0 },
{ 255, 219,   0 },{ 255, 218,   0 },{ 255, 217,   0 },{ 255, 216,   0 },{ 255, 215,   0 },
{ 255, 214,   0 },{ 255, 213,   0 },{ 255, 212,   0 },{ 255, 211,   0 },{ 255, 210,   0 },
{ 255, 209,   0 },{ 255, 208,   0 },{ 255, 207,   0 },{ 255, 206,   0 },{ 255, 205,   0 },
{ 255, 204,   0 },{ 255, 203,   0 },{ 255, 202,   0 },{ 255, 201,   0 },{ 255, 200,   0 },
{ 255, 199,   0 },{ 255, 198,   0 },{ 255, 197,   0 },{ 255, 196,   0 },{ 255, 195,   0 },
{ 255, 194,   0 },{ 255, 193,   0 },{ 255, 192,   0 },{ 255, 191,   0 },{ 255, 190,   0 },
{ 255, 189,   0 },{ 255, 188,   0 },{ 255, 187,   0 },{ 255, 186,   0 },{ 255, 185,   0 },
{ 255, 184,   0 },{ 255, 183,   0 },{ 255, 182,   0 },{ 255, 180,   0 },{ 255, 179,   0 },
{ 255, 178,   0 },{ 255, 177,   0 },{ 255, 176,   0 },{ 255, 176,   0 },{ 255, 175,   0 },
{ 255, 175,   0 },{ 255, 174,   0 },{ 255, 173,   0 },{ 255, 173,   0 },{ 255, 172,   0 },
{ 255, 171,   0 },{ 255, 171,   0 },{ 255, 170,   0 },{ 255, 169,   0 },{ 255, 168,   0 },
{ 255, 167,   0 },{ 255, 166,   0 },{ 255, 165,   0 },{ 255, 164,   0 },{ 255, 163,   0 },
{ 255, 162,   0 },{ 255, 161,   0 },{ 255, 160,   0 },{ 255, 159,   0 },{ 255, 158,   0 },
{ 255, 157,   0 },{ 255, 156,   0 },{ 255, 155,   0 },{ 255, 154,   0 },{ 255, 153,   0 },
{ 255, 152,   0 },{ 255, 151,   0 },{ 255, 150,   0 },{ 255, 150,   0 },{ 255, 149,   0 },
{ 255, 147,   0 },{ 255, 146,   0 },{ 255, 146,   0 },{ 255, 145,   0 },{ 255, 144,   0 },
{ 255, 143,   0 },{ 255, 142,   0 },{ 255, 141,   0 },{ 255, 140,   0 },{ 255, 139,   0 },
{ 255, 138,   0 },{ 255, 137,   0 },{ 255, 136,   0 },{ 255, 135,   0 },{ 255, 134,   0 },
{ 255, 133,   0 },{ 255, 132,   0 },{ 255, 131,   0 },{ 255, 130,   0 },{ 255, 129,   0 },
{ 255, 128,   0 },{ 255, 127,   0 },{ 255, 126,   0 },{ 255, 125,   0 },{ 255, 124,   0 },
{ 255, 123,   0 },{ 255, 122,   0 },{ 255, 121,   0 },{ 255, 120,   0 },{ 255, 119,   0 },
{ 255, 118,   0 },{ 255, 117,   0 },{ 255, 116,   0 },{ 255, 115,   0 },{ 255, 114,   0 },
{ 255, 113,   0 },{ 255, 112,   0 },{ 255, 111,   0 },{ 255, 109,   0 },{ 255, 108,   0 },
{ 255, 107,   0 },{ 255, 106,   0 },{ 255, 105,   0 },{ 255, 104,   0 },{ 255, 103,   0 },
{ 255, 102,   0 },{ 255, 101,   0 },{ 255, 100,   0 },{ 255,  99,   0 },{ 255,  98,   0 },
{ 255,  97,   0 },{ 255,  96,   0 },{ 255,  95,   0 },{ 255,  94,   0 },{ 255,  93,   0 },
{ 255,  92,   0 },{ 255,  91,   0 },{ 255,  91,   0 },{ 255,  90,   0 },{ 255,  90,   0 },
{ 255,  89,   0 },{ 255,  88,   0 },{ 255,  88,   0 },{ 255,  87,   0 },{ 255,  86,   0 },
{ 255,  86,   0 },{ 255,  85,   0 },{ 255,  84,   0 },{ 255,  83,   0 },{ 255,  82,   0 },
{ 255,  81,   0 },{ 255,  80,   0 },{ 255,  79,   0 },{ 255,  78,   0 },{ 255,  77,   0 },
{ 255,  76,   0 },{ 255,  75,   0 },{ 255,  74,   0 },{ 255,  73,   0 },{ 255,  72,   0 },
{ 255,  71,   0 },{ 255,  70,   0 },{ 255,  69,   0 },{ 255,  68,   0 },{ 255,  67,   0 },
{ 255,  66,   0 },{ 255,  65,   0 },{ 255,  64,   0 },{ 255,  63,   0 },{ 255,  62,   0 },
{ 255,  61,   0 },{ 255,  60,   0 },{ 255,  59,   0 },{ 255,  58,   0 },{ 255,  57,   0 },
{ 255,  56,   0 },{ 255,  55,   0 },{ 255,  54,   0 },{ 255,  54,   0 },{ 255,  53,   0 },
{ 255,  51,   0 },{ 255,  50,   0 },{ 255,  49,   0 },{ 255,  48,   0 },{ 255,  47,   0 },
{ 255,  46,   0 },{ 255,  45,   0 },{ 255,  44,   0 },{ 255,  43,   0 },{ 255,  42,   0 },
{ 255,  41,   0 },{ 255,  40,   0 },{ 255,  39,   0 },{ 255,  38,   0 },{ 255,  37,   0 },
{ 255,  36,   0 },{ 255,  35,   0 },{ 255,  34,   0 },{ 255,  33,   0 },{ 255,  32,   0 },
{ 255,  31,   0 },{ 255,  30,   0 },{ 255,  29,   0 },{ 255,  28,   0 },{ 255,  27,   0 },
{ 255,  26,   0 },{ 255,  25,   0 },{ 255,  24,   0 },{ 254,  23,   0 },{ 254,  22,   0 },
{ 254,  21,   0 },{ 254,  20,   0 },{ 254,  19,   0 },{ 254,  18,   0 },{ 253,  17,   0 },
{ 251,  16,   0 },{ 250,  15,   0 },{ 249,  14,   0 },{ 248,  13,   0 },{ 247,  12,   0 },
{ 246,  11,   0 },{ 244,  10,   0 },{ 243,   9,   0 },{ 242,   8,   0 },{ 241,   7,   0 },
{ 240,   6,   0 },{ 239,   6,   0 },{ 239,   5,   0 },{ 238,   4,   0 },{ 237,   4,   0 },
{ 236,   3,   0 },{ 235,   3,   0 },{ 234,   2,   0 },{ 234,   1,   0 },{ 233,   1,   0 },
{ 232,   0,   0 },{ 231,   0,   0 },{ 230,   0,   0 },{ 228,   0,   0 },{ 227,   0,   0 },
{ 226,   0,   0 },{ 225,   0,   0 },{ 223,   0,   0 },{ 222,   0,   0 },{ 221,   0,   0 },
{ 219,   0,   0 },{ 218,   0,   0 },{ 217,   0,   0 },{ 216,   0,   0 },{ 214,   0,   0 },
{ 213,   0,   0 },{ 212,   0,   0 },{ 211,   0,   0 },{ 209,   0,   0 },{ 208,   0,   0 },
{ 207,   0,   0 },{ 206,   0,   0 },{ 204,   0,   0 },{ 203,   0,   0 },{ 202,   0,   0 },
{ 201,   0,   0 },{ 200,   0,   0 },{ 198,   0,   0 },{ 197,   0,   0 },{ 196,   0,   0 },
{ 195,   0,   0 },{ 194,   0,   0 },{ 193,   0,   0 },{ 191,   0,   0 },{ 190,   0,   0 },
{ 189,   0,   0 },{ 188,   0,   0 },{ 186,   0,   0 },{ 185,   0,   0 },{ 184,   0,   0 },
{ 183,   0,   0 },{ 181,   0,   0 },{ 180,   0,   0 },{ 179,   0,   0 },{ 178,   0,   0 },
{ 176,   0,   0 },{ 175,   0,   0 },{ 174,   0,   0 },{ 173,   0,   0 },{ 171,   0,   0 },
{ 170,   0,   0 },{ 169,   0,   0 },{ 168,   0,   0 },{ 166,   0,   0 },{ 165,   0,   0 },
{ 164,   0,   0 },{ 163,   0,   0 },{ 161,   0,   0 },{ 160,   0,   0 },{ 159,   0,   0 },
{ 158,   0,   0 },{ 157,   0,   0 },{ 156,   0,   0 },{ 154,   0,   0 },{ 153,   0,   0 },
{ 152,   0,   0 },{ 151,   0,   0 },{ 150,   0,   0 },{ 148,   0,   0 },{ 147,   0,   0 },
{ 146,   0,   0 },{ 145,   0,   0 },{ 143,   0,   0 },{ 142,   0,   0 },{ 141,   0,   0 },
{ 140,   0,   0 },{ 138,   0,   0 },{ 137,   0,   0 },{ 136,   0,   0 },{ 135,   0,   0 },
{ 134,   0,   0 },{ 133,   0,   0 },{ 133,   0,   0 },{ 132,   0,   0 },{ 131,   0,   0 },
{ 130,   0,   0 },{ 129,   0,   0 },{ 129,   0,   0 },{ 128,   0,   0 },{ 127,   0,   0 },
{ 122,   0,   9 },{ 117,   0,  18 },{ 112,   0,  27 },{ 107,   0,  36 },{ 102,   0,  45 }
};

const rgb_t prism_colormap[1000] = {
    { 255,   0,   0 },{ 255,   2,   0 },{ 255,   4,   0 },{ 255,   6,   0 },{ 255,   8,   0 },
{ 255,  10,   0 },{ 255,  11,   0 },{ 255,  13,   0 },{ 255,  15,   0 },{ 255,  17,   0 },
{ 255,  19,   0 },{ 255,  21,   0 },{ 255,  23,   0 },{ 255,  25,   0 },{ 255,  27,   0 },
{ 255,  29,   0 },{ 255,  31,   0 },{ 255,  33,   0 },{ 255,  34,   0 },{ 255,  36,   0 },
{ 255,  38,   0 },{ 255,  40,   0 },{ 255,  42,   0 },{ 255,  44,   0 },{ 255,  46,   0 },
{ 255,  48,   0 },{ 255,  50,   0 },{ 255,  52,   0 },{ 255,  54,   0 },{ 255,  56,   0 },
{ 255,  57,   0 },{ 255,  59,   0 },{ 255,  61,   0 },{ 255,  63,   0 },{ 255,  65,   0 },
{ 255,  67,   0 },{ 255,  69,   0 },{ 255,  71,   0 },{ 255,  73,   0 },{ 255,  75,   0 },
{ 255,  77,   0 },{ 255,  78,   0 },{ 255,  80,   0 },{ 255,  82,   0 },{ 255,  84,   0 },
{ 255,  86,   0 },{ 255,  88,   0 },{ 255,  90,   0 },{ 255,  92,   0 },{ 255,  94,   0 },
{ 255,  96,   0 },{ 255,  98,   0 },{ 255, 100,   0 },{ 255, 101,   0 },{ 255, 103,   0 },
{ 255, 105,   0 },{ 255, 107,   0 },{ 255, 109,   0 },{ 255, 111,   0 },{ 255, 113,   0 },
{ 255, 115,   0 },{ 255, 117,   0 },{ 255, 119,   0 },{ 255, 121,   0 },{ 255, 123,   0 },
{ 255, 124,   0 },{ 255, 126,   0 },{ 255, 128,   0 },{ 255, 130,   0 },{ 255, 132,   0 },
{ 255, 134,   0 },{ 255, 136,   0 },{ 255, 138,   0 },{ 255, 140,   0 },{ 255, 142,   0 },
{ 255, 144,   0 },{ 255, 145,   0 },{ 255, 147,   0 },{ 255, 149,   0 },{ 255, 151,   0 },
{ 255, 153,   0 },{ 255, 155,   0 },{ 255, 157,   0 },{ 255, 159,   0 },{ 255, 161,   0 },
{ 255, 163,   0 },{ 255, 165,   0 },{ 255, 167,   0 },{ 255, 168,   0 },{ 255, 170,   0 },
{ 255, 172,   0 },{ 255, 174,   0 },{ 255, 176,   0 },{ 255, 178,   0 },{ 255, 180,   0 },
{ 255, 182,   0 },{ 255, 184,   0 },{ 255, 186,   0 },{ 255, 188,   0 },{ 255, 190,   0 },
{ 255, 191,   0 },{ 255, 193,   0 },{ 255, 195,   0 },{ 255, 197,   0 },{ 255, 199,   0 },
{ 255, 201,   0 },{ 255, 203,   0 },{ 255, 205,   0 },{ 255, 207,   0 },{ 255, 209,   0 },
{ 255, 211,   0 },{ 255, 212,   0 },{ 255, 214,   0 },{ 255, 216,   0 },{ 255, 218,   0 },
{ 255, 220,   0 },{ 255, 222,   0 },{ 255, 224,   0 },{ 255, 226,   0 },{ 255, 228,   0 },
{ 255, 230,   0 },{ 255, 232,   0 },{ 255, 234,   0 },{ 255, 235,   0 },{ 255, 237,   0 },
{ 255, 239,   0 },{ 255, 241,   0 },{ 255, 243,   0 },{ 255, 245,   0 },{ 255, 247,   0 },
{ 255, 249,   0 },{ 255, 251,   0 },{ 255, 253,   0 },{ 255, 255,   0 },{ 252, 255,   0 },
{ 248, 255,   0 },{ 244, 255,   0 },{ 240, 255,   0 },{ 237, 255,   0 },{ 233, 255,   0 },
{ 229, 255,   0 },{ 225, 255,   0 },{ 221, 255,   0 },{ 217, 255,   0 },{ 214, 255,   0 },
{ 210, 255,   0 },{ 206, 255,   0 },{ 202, 255,   0 },{ 198, 255,   0 },{ 195, 255,   0 },
{ 191, 255,   0 },{ 187, 255,   0 },{ 183, 255,   0 },{ 179, 255,   0 },{ 175, 255,   0 },
{ 172, 255,   0 },{ 168, 255,   0 },{ 164, 255,   0 },{ 160, 255,   0 },{ 156, 255,   0 },
{ 152, 255,   0 },{ 149, 255,   0 },{ 145, 255,   0 },{ 141, 255,   0 },{ 137, 255,   0 },
{ 133, 255,   0 },{ 129, 255,   0 },{ 126, 255,   0 },{ 122, 255,   0 },{ 118, 255,   0 },
{ 114, 255,   0 },{ 110, 255,   0 },{ 106, 255,   0 },{ 103, 255,   0 },{ 99, 255,   0 },
{ 95, 255,   0 },{ 91, 255,   0 },{ 87, 255,   0 },{ 83, 255,   0 },{ 80, 255,   0 },
{ 76, 255,   0 },{ 72, 255,   0 },{ 68, 255,   0 },{ 64, 255,   0 },{ 60, 255,   0 },
{ 57, 255,   0 },{ 53, 255,   0 },{ 49, 255,   0 },{ 45, 255,   0 },{ 41, 255,   0 },
{ 38, 255,   0 },{ 34, 255,   0 },{ 30, 255,   0 },{ 26, 255,   0 },{ 22, 255,   0 },
{ 18, 255,   0 },{ 15, 255,   0 },{ 11, 255,   0 },{ 7, 255,   0 },{ 3, 255,   0 },
{ 0, 254,   1 },{ 0, 250,   5 },{ 0, 247,   8 },{ 0, 243,  12 },{ 0, 239,  16 },
{ 0, 235,  20 },{ 0, 231,  24 },{ 0, 227,  28 },{ 0, 224,  31 },{ 0, 220,  35 },
{ 0, 216,  39 },{ 0, 212,  43 },{ 0, 208,  47 },{ 0, 204,  51 },{ 0, 201,  54 },
{ 0, 197,  58 },{ 0, 193,  62 },{ 0, 189,  66 },{ 0, 185,  70 },{ 0, 181,  74 },
{ 0, 178,  77 },{ 0, 174,  81 },{ 0, 170,  85 },{ 0, 166,  89 },{ 0, 162,  93 },
{ 0, 159,  96 },{ 0, 155, 100 },{ 0, 151, 104 },{ 0, 147, 108 },{ 0, 143, 112 },
{ 0, 139, 116 },{ 0, 136, 119 },{ 0, 132, 123 },{ 0, 128, 127 },{ 0, 124, 131 },
{ 0, 120, 135 },{ 0, 116, 139 },{ 0, 113, 142 },{ 0, 109, 146 },{ 0, 105, 150 },
{ 0, 101, 154 },{ 0,  97, 158 },{ 0,  93, 162 },{ 0,  90, 165 },{ 0,  86, 169 },
{ 0,  82, 173 },{ 0,  78, 177 },{ 0,  74, 181 },{ 0,  70, 185 },{ 0,  67, 188 },
{ 0,  63, 192 },{ 0,  59, 196 },{ 0,  55, 200 },{ 0,  51, 204 },{ 0,  47, 208 },
{ 0,  44, 211 },{ 0,  40, 215 },{ 0,  36, 219 },{ 0,  32, 223 },{ 0,  28, 227 },
{ 0,  25, 230 },{ 0,  21, 234 },{ 0,  17, 238 },{ 0,  13, 242 },{ 0,   9, 246 },
{ 0,   5, 250 },{ 0,   2, 253 },{ 2,   0, 255 },{ 4,   0, 255 },{ 7,   0, 255 },
{ 9,   0, 255 },{ 12,   0, 255 },{ 14,   0, 255 },{ 17,   0, 255 },{ 19,   0, 255 },
{ 22,   0, 255 },{ 25,   0, 255 },{ 27,   0, 255 },{ 30,   0, 255 },{ 32,   0, 255 },
{ 35,   0, 255 },{ 37,   0, 255 },{ 40,   0, 255 },{ 42,   0, 255 },{ 45,   0, 255 },
{ 47,   0, 255 },{ 50,   0, 255 },{ 53,   0, 255 },{ 55,   0, 255 },{ 58,   0, 255 },
{ 60,   0, 255 },{ 63,   0, 255 },{ 65,   0, 255 },{ 68,   0, 255 },{ 70,   0, 255 },
{ 73,   0, 255 },{ 76,   0, 255 },{ 78,   0, 255 },{ 81,   0, 255 },{ 83,   0, 255 },
{ 86,   0, 255 },{ 88,   0, 255 },{ 91,   0, 255 },{ 93,   0, 255 },{ 96,   0, 255 },
{ 99,   0, 255 },{ 101,   0, 255 },{ 104,   0, 255 },{ 106,   0, 255 },{ 109,   0, 255 },
{ 111,   0, 255 },{ 114,   0, 255 },{ 116,   0, 255 },{ 119,   0, 255 },{ 122,   0, 255 },
{ 124,   0, 255 },{ 127,   0, 255 },{ 129,   0, 255 },{ 132,   0, 255 },{ 134,   0, 255 },
{ 137,   0, 255 },{ 139,   0, 255 },{ 142,   0, 255 },{ 144,   0, 255 },{ 147,   0, 255 },
{ 150,   0, 255 },{ 152,   0, 255 },{ 155,   0, 255 },{ 157,   0, 255 },{ 160,   0, 255 },
{ 162,   0, 255 },{ 165,   0, 255 },{ 167,   0, 255 },{ 170,   0, 255 },{ 171,   0, 251 },
{ 173,   0, 247 },{ 174,   0, 244 },{ 175,   0, 240 },{ 176,   0, 236 },{ 178,   0, 232 },
{ 179,   0, 228 },{ 180,   0, 224 },{ 181,   0, 221 },{ 183,   0, 217 },{ 184,   0, 213 },
{ 185,   0, 209 },{ 187,   0, 205 },{ 188,   0, 201 },{ 189,   0, 198 },{ 190,   0, 194 },
{ 192,   0, 190 },{ 193,   0, 186 },{ 194,   0, 182 },{ 196,   0, 178 },{ 197,   0, 175 },
{ 198,   0, 171 },{ 199,   0, 167 },{ 201,   0, 163 },{ 202,   0, 159 },{ 203,   0, 155 },
{ 204,   0, 152 },{ 206,   0, 148 },{ 207,   0, 144 },{ 208,   0, 140 },{ 210,   0, 136 },
{ 211,   0, 132 },{ 212,   0, 129 },{ 213,   0, 125 },{ 215,   0, 121 },{ 216,   0, 117 },
{ 217,   0, 113 },{ 218,   0, 110 },{ 220,   0, 106 },{ 221,   0, 102 },{ 222,   0,  98 },
{ 224,   0,  94 },{ 225,   0,  90 },{ 226,   0,  87 },{ 227,   0,  83 },{ 229,   0,  79 },
{ 230,   0,  75 },{ 231,   0,  71 },{ 233,   0,  67 },{ 234,   0,  64 },{ 235,   0,  60 },
{ 236,   0,  56 },{ 238,   0,  52 },{ 239,   0,  48 },{ 240,   0,  44 },{ 241,   0,  41 },
{ 243,   0,  37 },{ 244,   0,  33 },{ 245,   0,  29 },{ 247,   0,  25 },{ 248,   0,  21 },
{ 249,   0,  18 },{ 250,   0,  14 },{ 252,   0,  10 },{ 253,   0,   6 },{ 254,   0,   2 },
{ 255,   1,   0 },{ 255,   3,   0 },{ 255,   5,   0 },{ 255,   7,   0 },{ 255,   8,   0 },
{ 255,  10,   0 },{ 255,  12,   0 },{ 255,  14,   0 },{ 255,  16,   0 },{ 255,  18,   0 },
{ 255,  20,   0 },{ 255,  22,   0 },{ 255,  24,   0 },{ 255,  26,   0 },{ 255,  28,   0 },
{ 255,  29,   0 },{ 255,  31,   0 },{ 255,  33,   0 },{ 255,  35,   0 },{ 255,  37,   0 },
{ 255,  39,   0 },{ 255,  41,   0 },{ 255,  43,   0 },{ 255,  45,   0 },{ 255,  47,   0 },
{ 255,  49,   0 },{ 255,  51,   0 },{ 255,  52,   0 },{ 255,  54,   0 },{ 255,  56,   0 },
{ 255,  58,   0 },{ 255,  60,   0 },{ 255,  62,   0 },{ 255,  64,   0 },{ 255,  66,   0 },
{ 255,  68,   0 },{ 255,  70,   0 },{ 255,  72,   0 },{ 255,  74,   0 },{ 255,  75,   0 },
{ 255,  77,   0 },{ 255,  79,   0 },{ 255,  81,   0 },{ 255,  83,   0 },{ 255,  85,   0 },
{ 255,  87,   0 },{ 255,  89,   0 },{ 255,  91,   0 },{ 255,  93,   0 },{ 255,  95,   0 },
{ 255,  96,   0 },{ 255,  98,   0 },{ 255, 100,   0 },{ 255, 102,   0 },{ 255, 104,   0 },
{ 255, 106,   0 },{ 255, 108,   0 },{ 255, 110,   0 },{ 255, 112,   0 },{ 255, 114,   0 },
{ 255, 116,   0 },{ 255, 118,   0 },{ 255, 119,   0 },{ 255, 121,   0 },{ 255, 123,   0 },
{ 255, 125,   0 },{ 255, 127,   0 },{ 255, 129,   0 },{ 255, 131,   0 },{ 255, 133,   0 },
{ 255, 135,   0 },{ 255, 137,   0 },{ 255, 139,   0 },{ 255, 141,   0 },{ 255, 142,   0 },
{ 255, 144,   0 },{ 255, 146,   0 },{ 255, 148,   0 },{ 255, 150,   0 },{ 255, 152,   0 },
{ 255, 154,   0 },{ 255, 156,   0 },{ 255, 158,   0 },{ 255, 160,   0 },{ 255, 162,   0 },
{ 255, 163,   0 },{ 255, 165,   0 },{ 255, 167,   0 },{ 255, 169,   0 },{ 255, 171,   0 },
{ 255, 173,   0 },{ 255, 175,   0 },{ 255, 177,   0 },{ 255, 179,   0 },{ 255, 181,   0 },
{ 255, 183,   0 },{ 255, 185,   0 },{ 255, 186,   0 },{ 255, 188,   0 },{ 255, 190,   0 },
{ 255, 192,   0 },{ 255, 194,   0 },{ 255, 196,   0 },{ 255, 198,   0 },{ 255, 200,   0 },
{ 255, 202,   0 },{ 255, 204,   0 },{ 255, 206,   0 },{ 255, 208,   0 },{ 255, 209,   0 },
{ 255, 211,   0 },{ 255, 213,   0 },{ 255, 215,   0 },{ 255, 217,   0 },{ 255, 219,   0 },
{ 255, 221,   0 },{ 255, 223,   0 },{ 255, 225,   0 },{ 255, 227,   0 },{ 255, 229,   0 },
{ 255, 230,   0 },{ 255, 232,   0 },{ 255, 234,   0 },{ 255, 236,   0 },{ 255, 238,   0 },
{ 255, 240,   0 },{ 255, 242,   0 },{ 255, 244,   0 },{ 255, 246,   0 },{ 255, 248,   0 },
{ 255, 250,   0 },{ 255, 252,   0 },{ 255, 253,   0 },{ 254, 255,   0 },{ 250, 255,   0 },
{ 247, 255,   0 },{ 243, 255,   0 },{ 239, 255,   0 },{ 235, 255,   0 },{ 231, 255,   0 },
{ 227, 255,   0 },{ 224, 255,   0 },{ 220, 255,   0 },{ 216, 255,   0 },{ 212, 255,   0 },
{ 208, 255,   0 },{ 204, 255,   0 },{ 201, 255,   0 },{ 197, 255,   0 },{ 193, 255,   0 },
{ 189, 255,   0 },{ 185, 255,   0 },{ 181, 255,   0 },{ 178, 255,   0 },{ 174, 255,   0 },
{ 170, 255,   0 },{ 166, 255,   0 },{ 162, 255,   0 },{ 159, 255,   0 },{ 155, 255,   0 },
{ 151, 255,   0 },{ 147, 255,   0 },{ 143, 255,   0 },{ 139, 255,   0 },{ 136, 255,   0 },
{ 132, 255,   0 },{ 128, 255,   0 },{ 124, 255,   0 },{ 120, 255,   0 },{ 116, 255,   0 },
{ 113, 255,   0 },{ 109, 255,   0 },{ 105, 255,   0 },{ 101, 255,   0 },{ 97, 255,   0 },
{ 93, 255,   0 },{ 90, 255,   0 },{ 86, 255,   0 },{ 82, 255,   0 },{ 78, 255,   0 },
{ 74, 255,   0 },{ 70, 255,   0 },{ 67, 255,   0 },{ 63, 255,   0 },{ 59, 255,   0 },
{ 55, 255,   0 },{ 51, 255,   0 },{ 47, 255,   0 },{ 44, 255,   0 },{ 40, 255,   0 },
{ 36, 255,   0 },{ 32, 255,   0 },{ 28, 255,   0 },{ 25, 255,   0 },{ 21, 255,   0 },
{ 17, 255,   0 },{ 13, 255,   0 },{ 9, 255,   0 },{ 5, 255,   0 },{ 2, 255,   0 },
{ 0, 253,   2 },{ 0, 249,   6 },{ 0, 245,  10 },{ 0, 241,  14 },{ 0, 237,  18 },
{ 0, 234,  21 },{ 0, 230,  25 },{ 0, 226,  29 },{ 0, 222,  33 },{ 0, 218,  37 },
{ 0, 214,  41 },{ 0, 211,  44 },{ 0, 207,  48 },{ 0, 203,  52 },{ 0, 199,  56 },
{ 0, 195,  60 },{ 0, 191,  64 },{ 0, 188,  67 },{ 0, 184,  71 },{ 0, 180,  75 },
{ 0, 176,  79 },{ 0, 172,  83 },{ 0, 168,  87 },{ 0, 165,  90 },{ 0, 161,  94 },
{ 0, 157,  98 },{ 0, 153, 102 },{ 0, 149, 106 },{ 0, 145, 110 },{ 0, 142, 113 },
{ 0, 138, 117 },{ 0, 134, 121 },{ 0, 130, 125 },{ 0, 126, 129 },{ 0, 123, 132 },
{ 0, 119, 136 },{ 0, 115, 140 },{ 0, 111, 144 },{ 0, 107, 148 },{ 0, 103, 152 },
{ 0, 100, 155 },{ 0,  96, 159 },{ 0,  92, 163 },{ 0,  88, 167 },{ 0,  84, 171 },
{ 0,  80, 175 },{ 0,  77, 178 },{ 0,  73, 182 },{ 0,  69, 186 },{ 0,  65, 190 },
{ 0,  61, 194 },{ 0,  57, 198 },{ 0,  54, 201 },{ 0,  50, 205 },{ 0,  46, 209 },
{ 0,  42, 213 },{ 0,  38, 217 },{ 0,  34, 221 },{ 0,  31, 224 },{ 0,  27, 228 },
{ 0,  23, 232 },{ 0,  19, 236 },{ 0,  15, 240 },{ 0,  11, 244 },{ 0,   8, 247 },
{ 0,   4, 251 },{ 0,   0, 255 },{ 3,   0, 255 },{ 5,   0, 255 },{ 8,   0, 255 },
{ 10,   0, 255 },{ 13,   0, 255 },{ 15,   0, 255 },{ 18,   0, 255 },{ 20,   0, 255 },
{ 23,   0, 255 },{ 26,   0, 255 },{ 28,   0, 255 },{ 31,   0, 255 },{ 33,   0, 255 },
{ 36,   0, 255 },{ 38,   0, 255 },{ 41,   0, 255 },{ 43,   0, 255 },{ 46,   0, 255 },
{ 48,   0, 255 },{ 51,   0, 255 },{ 54,   0, 255 },{ 56,   0, 255 },{ 59,   0, 255 },
{ 61,   0, 255 },{ 64,   0, 255 },{ 66,   0, 255 },{ 69,   0, 255 },{ 71,   0, 255 },
{ 74,   0, 255 },{ 77,   0, 255 },{ 79,   0, 255 },{ 82,   0, 255 },{ 84,   0, 255 },
{ 87,   0, 255 },{ 89,   0, 255 },{ 92,   0, 255 },{ 94,   0, 255 },{ 97,   0, 255 },
{ 100,   0, 255 },{ 102,   0, 255 },{ 105,   0, 255 },{ 107,   0, 255 },{ 110,   0, 255 },
{ 112,   0, 255 },{ 115,   0, 255 },{ 117,   0, 255 },{ 120,   0, 255 },{ 123,   0, 255 },
{ 125,   0, 255 },{ 128,   0, 255 },{ 130,   0, 255 },{ 133,   0, 255 },{ 135,   0, 255 },
{ 138,   0, 255 },{ 140,   0, 255 },{ 143,   0, 255 },{ 145,   0, 255 },{ 148,   0, 255 },
{ 151,   0, 255 },{ 153,   0, 255 },{ 156,   0, 255 },{ 158,   0, 255 },{ 161,   0, 255 },
{ 163,   0, 255 },{ 166,   0, 255 },{ 168,   0, 255 },{ 171,   0, 253 },{ 172,   0, 250 },
{ 173,   0, 246 },{ 174,   0, 242 },{ 176,   0, 238 },{ 177,   0, 234 },{ 178,   0, 230 },
{ 179,   0, 227 },{ 181,   0, 223 },{ 182,   0, 219 },{ 183,   0, 215 },{ 185,   0, 211 },
{ 186,   0, 208 },{ 187,   0, 204 },{ 188,   0, 200 },{ 190,   0, 196 },{ 191,   0, 192 },
{ 192,   0, 188 },{ 193,   0, 185 },{ 195,   0, 181 },{ 196,   0, 177 },{ 197,   0, 173 },
{ 199,   0, 169 },{ 200,   0, 165 },{ 201,   0, 162 },{ 202,   0, 158 },{ 204,   0, 154 },
{ 205,   0, 150 },{ 206,   0, 146 },{ 208,   0, 142 },{ 209,   0, 139 },{ 210,   0, 135 },
{ 211,   0, 131 },{ 213,   0, 127 },{ 214,   0, 123 },{ 215,   0, 119 },{ 216,   0, 116 },
{ 218,   0, 112 },{ 219,   0, 108 },{ 220,   0, 104 },{ 222,   0, 100 },{ 223,   0,  96 },
{ 224,   0,  93 },{ 225,   0,  89 },{ 227,   0,  85 },{ 228,   0,  81 },{ 229,   0,  77 },
{ 230,   0,  74 },{ 232,   0,  70 },{ 233,   0,  66 },{ 234,   0,  62 },{ 236,   0,  58 },
{ 237,   0,  54 },{ 238,   0,  51 },{ 239,   0,  47 },{ 241,   0,  43 },{ 242,   0,  39 },
{ 243,   0,  35 },{ 245,   0,  31 },{ 246,   0,  28 },{ 247,   0,  24 },{ 248,   0,  20 },
{ 250,   0,  16 },{ 251,   0,  12 },{ 252,   0,   8 },{ 253,   0,   5 },{ 255,   0,   1 },
{ 255,   2,   0 },{ 255,   3,   0 },{ 255,   5,   0 },{ 255,   7,   0 },{ 255,   9,   0 },
{ 255,  11,   0 },{ 255,  13,   0 },{ 255,  15,   0 },{ 255,  17,   0 },{ 255,  19,   0 },
{ 255,  21,   0 },{ 255,  23,   0 },{ 255,  25,   0 },{ 255,  26,   0 },{ 255,  28,   0 },
{ 255,  30,   0 },{ 255,  32,   0 },{ 255,  34,   0 },{ 255,  36,   0 },{ 255,  38,   0 },
{ 255,  40,   0 },{ 255,  42,   0 },{ 255,  44,   0 },{ 255,  46,   0 },{ 255,  47,   0 },
{ 255,  49,   0 },{ 255,  51,   0 },{ 255,  53,   0 },{ 255,  55,   0 },{ 255,  57,   0 },
{ 255,  59,   0 },{ 255,  61,   0 },{ 255,  63,   0 },{ 255,  65,   0 },{ 255,  67,   0 },
{ 255,  69,   0 },{ 255,  70,   0 },{ 255,  72,   0 },{ 255,  74,   0 },{ 255,  76,   0 },
{ 255,  78,   0 },{ 255,  80,   0 },{ 255,  82,   0 },{ 255,  84,   0 },{ 255,  86,   0 },
{ 255,  88,   0 },{ 255,  90,   0 },{ 255,  92,   0 },{ 255,  93,   0 },{ 255,  95,   0 },
{ 255,  97,   0 },{ 255,  99,   0 },{ 255, 101,   0 },{ 255, 103,   0 },{ 255, 105,   0 },
{ 255, 107,   0 },{ 255, 109,   0 },{ 255, 111,   0 },{ 255, 113,   0 },{ 255, 114,   0 },
{ 255, 116,   0 },{ 255, 118,   0 },{ 255, 120,   0 },{ 255, 122,   0 },{ 255, 124,   0 },
{ 255, 126,   0 },{ 255, 128,   0 },{ 255, 130,   0 },{ 255, 132,   0 },{ 255, 134,   0 },
{ 255, 136,   0 },{ 255, 137,   0 },{ 255, 139,   0 },{ 255, 141,   0 },{ 255, 143,   0 },
{ 255, 145,   0 },{ 255, 147,   0 },{ 255, 149,   0 },{ 255, 151,   0 },{ 255, 153,   0 },
{ 255, 155,   0 },{ 255, 157,   0 },{ 255, 159,   0 },{ 255, 160,   0 },{ 255, 162,   0 },
{ 255, 164,   0 },{ 255, 166,   0 },{ 255, 168,   0 },{ 255, 170,   0 },{ 255, 172,   0 },
{ 255, 174,   0 },{ 255, 176,   0 },{ 255, 178,   0 },{ 255, 180,   0 },{ 255, 181,   0 },
{ 255, 183,   0 },{ 255, 185,   0 },{ 255, 187,   0 },{ 255, 189,   0 },{ 255, 191,   0 },
{ 255, 193,   0 },{ 255, 195,   0 },{ 255, 197,   0 },{ 255, 199,   0 },{ 255, 201,   0 },
{ 255, 203,   0 },{ 255, 204,   0 },{ 255, 206,   0 },{ 255, 208,   0 },{ 255, 210,   0 },
{ 255, 212,   0 },{ 255, 214,   0 },{ 255, 216,   0 },{ 255, 218,   0 },{ 255, 220,   0 },
{ 255, 222,   0 },{ 255, 224,   0 },{ 255, 226,   0 },{ 255, 227,   0 },{ 255, 229,   0 },
{ 255, 231,   0 },{ 255, 233,   0 },{ 255, 235,   0 },{ 255, 237,   0 },{ 255, 239,   0 },
{ 255, 241,   0 },{ 255, 243,   0 },{ 255, 245,   0 },{ 255, 247,   0 },{ 255, 248,   0 },
{ 255, 250,   0 },{ 255, 252,   0 },{ 255, 254,   0 },{ 253, 255,   0 },{ 249, 255,   0 },
{ 245, 255,   0 },{ 241, 255,   0 },{ 237, 255,   0 },{ 234, 255,   0 },{ 230, 255,   0 },
{ 226, 255,   0 },{ 222, 255,   0 },{ 218, 255,   0 },{ 214, 255,   0 },{ 211, 255,   0 },
{ 207, 255,   0 },{ 203, 255,   0 },{ 199, 255,   0 },{ 195, 255,   0 },{ 191, 255,   0 },
{ 188, 255,   0 },{ 184, 255,   0 },{ 180, 255,   0 },{ 176, 255,   0 },{ 172, 255,   0 },
{ 168, 255,   0 },{ 165, 255,   0 },{ 161, 255,   0 },{ 157, 255,   0 },{ 153, 255,   0 },
{ 149, 255,   0 },{ 145, 255,   0 },{ 142, 255,   0 },{ 138, 255,   0 },{ 134, 255,   0 },
{ 130, 255,   0 },{ 126, 255,   0 },{ 123, 255,   0 },{ 119, 255,   0 },{ 115, 255,   0 },
{ 111, 255,   0 },{ 107, 255,   0 },{ 103, 255,   0 },{ 100, 255,   0 },{ 96, 255,   0 },
{ 92, 255,   0 },{ 88, 255,   0 },{ 84, 255,   0 },{ 80, 255,   0 },{ 77, 255,   0 },
{ 73, 255,   0 },{ 69, 255,   0 },{ 65, 255,   0 },{ 61, 255,   0 },{ 57, 255,   0 },
{ 54, 255,   0 },{ 50, 255,   0 },{ 46, 255,   0 },{ 42, 255,   0 },{ 38, 255,   0 },
{ 34, 255,   0 },{ 31, 255,   0 },{ 27, 255,   0 },{ 23, 255,   0 },{ 19, 255,   0 },
{ 15, 255,   0 },{ 11, 255,   0 },{ 8, 255,   0 },{ 4, 255,   0 },{ 0, 255,   0 }
};

const rgb_t vga_colormap[1000] = {
    { 255, 255, 255 },{ 254, 254, 254 },{ 253, 253, 253 },{ 252, 252, 252 },{ 251, 251, 251 },
{ 250, 250, 250 },{ 249, 249, 249 },{ 248, 248, 248 },{ 247, 247, 247 },{ 246, 246, 246 },
{ 245, 245, 245 },{ 244, 244, 244 },{ 244, 244, 244 },{ 243, 243, 243 },{ 242, 242, 242 },
{ 241, 241, 241 },{ 240, 240, 240 },{ 239, 239, 239 },{ 238, 238, 238 },{ 237, 237, 237 },
{ 236, 236, 236 },{ 235, 235, 235 },{ 234, 234, 234 },{ 233, 233, 233 },{ 232, 232, 232 },
{ 231, 231, 231 },{ 230, 230, 230 },{ 229, 229, 229 },{ 228, 228, 228 },{ 227, 227, 227 },
{ 226, 226, 226 },{ 225, 225, 225 },{ 224, 224, 224 },{ 223, 223, 223 },{ 222, 222, 222 },
{ 221, 221, 221 },{ 221, 221, 221 },{ 220, 220, 220 },{ 219, 219, 219 },{ 218, 218, 218 },
{ 217, 217, 217 },{ 216, 216, 216 },{ 215, 215, 215 },{ 214, 214, 214 },{ 213, 213, 213 },
{ 212, 212, 212 },{ 211, 211, 211 },{ 210, 210, 210 },{ 209, 209, 209 },{ 208, 208, 208 },
{ 207, 207, 207 },{ 206, 206, 206 },{ 205, 205, 205 },{ 204, 204, 204 },{ 203, 203, 203 },
{ 202, 202, 202 },{ 201, 201, 201 },{ 200, 200, 200 },{ 199, 199, 199 },{ 199, 199, 199 },
{ 198, 198, 198 },{ 197, 197, 197 },{ 196, 196, 196 },{ 195, 195, 195 },{ 194, 194, 194 },
{ 193, 193, 193 },{ 192, 192, 192 },{ 192, 190, 190 },{ 193, 187, 187 },{ 194, 184, 184 },
{ 195, 181, 181 },{ 195, 179, 179 },{ 196, 176, 176 },{ 197, 173, 173 },{ 198, 170, 170 },
{ 199, 167, 167 },{ 200, 164, 164 },{ 201, 161, 161 },{ 202, 159, 159 },{ 203, 156, 156 },
{ 204, 153, 153 },{ 205, 150, 150 },{ 206, 147, 147 },{ 207, 144, 144 },{ 208, 141, 141 },
{ 209, 138, 138 },{ 210, 136, 136 },{ 211, 133, 133 },{ 212, 130, 130 },{ 213, 127, 127 },
{ 214, 124, 124 },{ 215, 121, 121 },{ 216, 118, 118 },{ 217, 115, 115 },{ 217, 113, 113 },
{ 218, 110, 110 },{ 219, 107, 107 },{ 220, 104, 104 },{ 221, 101, 101 },{ 222,  98,  98 },
{ 223,  95,  95 },{ 224,  92,  92 },{ 225,  90,  90 },{ 226,  87,  87 },{ 227,  84,  84 },
{ 228,  81,  81 },{ 229,  78,  78 },{ 230,  75,  75 },{ 231,  72,  72 },{ 232,  69,  69 },
{ 233,  67,  67 },{ 234,  64,  64 },{ 235,  61,  61 },{ 236,  58,  58 },{ 237,  55,  55 },
{ 238,  52,  52 },{ 239,  49,  49 },{ 239,  47,  47 },{ 240,  44,  44 },{ 241,  41,  41 },
{ 242,  38,  38 },{ 243,  35,  35 },{ 244,  32,  32 },{ 245,  29,  29 },{ 246,  26,  26 },
{ 247,  24,  24 },{ 248,  21,  21 },{ 249,  18,  18 },{ 250,  15,  15 },{ 251,  12,  12 },
{ 252,   9,   9 },{ 253,   6,   6 },{ 254,   3,   3 },{ 255,   1,   1 },{ 255,   3,   0 },
{ 255,   7,   0 },{ 255,  11,   0 },{ 255,  15,   0 },{ 255,  18,   0 },{ 255,  22,   0 },
{ 255,  26,   0 },{ 255,  30,   0 },{ 255,  34,   0 },{ 255,  38,   0 },{ 255,  41,   0 },
{ 255,  45,   0 },{ 255,  49,   0 },{ 255,  53,   0 },{ 255,  57,   0 },{ 255,  60,   0 },
{ 255,  64,   0 },{ 255,  68,   0 },{ 255,  72,   0 },{ 255,  76,   0 },{ 255,  80,   0 },
{ 255,  83,   0 },{ 255,  87,   0 },{ 255,  91,   0 },{ 255,  95,   0 },{ 255,  99,   0 },
{ 255, 103,   0 },{ 255, 106,   0 },{ 255, 110,   0 },{ 255, 114,   0 },{ 255, 118,   0 },
{ 255, 122,   0 },{ 255, 126,   0 },{ 255, 129,   0 },{ 255, 133,   0 },{ 255, 137,   0 },
{ 255, 141,   0 },{ 255, 145,   0 },{ 255, 149,   0 },{ 255, 152,   0 },{ 255, 156,   0 },
{ 255, 160,   0 },{ 255, 164,   0 },{ 255, 168,   0 },{ 255, 172,   0 },{ 255, 175,   0 },
{ 255, 179,   0 },{ 255, 183,   0 },{ 255, 187,   0 },{ 255, 191,   0 },{ 255, 195,   0 },
{ 255, 198,   0 },{ 255, 202,   0 },{ 255, 206,   0 },{ 255, 210,   0 },{ 255, 214,   0 },
{ 255, 217,   0 },{ 255, 221,   0 },{ 255, 225,   0 },{ 255, 229,   0 },{ 255, 233,   0 },
{ 255, 237,   0 },{ 255, 240,   0 },{ 255, 244,   0 },{ 255, 248,   0 },{ 255, 252,   0 },
{ 254, 255,   0 },{ 250, 255,   0 },{ 247, 255,   0 },{ 243, 255,   0 },{ 239, 255,   0 },
{ 235, 255,   0 },{ 231, 255,   0 },{ 227, 255,   0 },{ 224, 255,   0 },{ 220, 255,   0 },
{ 216, 255,   0 },{ 212, 255,   0 },{ 208, 255,   0 },{ 204, 255,   0 },{ 201, 255,   0 },
{ 197, 255,   0 },{ 193, 255,   0 },{ 189, 255,   0 },{ 185, 255,   0 },{ 181, 255,   0 },
{ 178, 255,   0 },{ 174, 255,   0 },{ 170, 255,   0 },{ 166, 255,   0 },{ 162, 255,   0 },
{ 159, 255,   0 },{ 155, 255,   0 },{ 151, 255,   0 },{ 147, 255,   0 },{ 143, 255,   0 },
{ 139, 255,   0 },{ 136, 255,   0 },{ 132, 255,   0 },{ 128, 255,   0 },{ 124, 255,   0 },
{ 120, 255,   0 },{ 116, 255,   0 },{ 113, 255,   0 },{ 109, 255,   0 },{ 105, 255,   0 },
{ 101, 255,   0 },{ 97, 255,   0 },{ 93, 255,   0 },{ 90, 255,   0 },{ 86, 255,   0 },
{ 82, 255,   0 },{ 78, 255,   0 },{ 74, 255,   0 },{ 70, 255,   0 },{ 67, 255,   0 },
{ 63, 255,   0 },{ 59, 255,   0 },{ 55, 255,   0 },{ 51, 255,   0 },{ 47, 255,   0 },
{ 44, 255,   0 },{ 40, 255,   0 },{ 36, 255,   0 },{ 32, 255,   0 },{ 28, 255,   0 },
{ 25, 255,   0 },{ 21, 255,   0 },{ 17, 255,   0 },{ 13, 255,   0 },{ 9, 255,   0 },
{ 5, 255,   0 },{ 2, 255,   0 },{ 0, 255,   2 },{ 0, 255,   6 },{ 0, 255,  10 },
{ 0, 255,  14 },{ 0, 255,  18 },{ 0, 255,  21 },{ 0, 255,  25 },{ 0, 255,  29 },
{ 0, 255,  33 },{ 0, 255,  37 },{ 0, 255,  41 },{ 0, 255,  44 },{ 0, 255,  48 },
{ 0, 255,  52 },{ 0, 255,  56 },{ 0, 255,  60 },{ 0, 255,  64 },{ 0, 255,  67 },
{ 0, 255,  71 },{ 0, 255,  75 },{ 0, 255,  79 },{ 0, 255,  83 },{ 0, 255,  87 },
{ 0, 255,  90 },{ 0, 255,  94 },{ 0, 255,  98 },{ 0, 255, 102 },{ 0, 255, 106 },
{ 0, 255, 110 },{ 0, 255, 113 },{ 0, 255, 117 },{ 0, 255, 121 },{ 0, 255, 125 },
{ 0, 255, 129 },{ 0, 255, 132 },{ 0, 255, 136 },{ 0, 255, 140 },{ 0, 255, 144 },
{ 0, 255, 148 },{ 0, 255, 152 },{ 0, 255, 155 },{ 0, 255, 159 },{ 0, 255, 163 },
{ 0, 255, 167 },{ 0, 255, 171 },{ 0, 255, 175 },{ 0, 255, 178 },{ 0, 255, 182 },
{ 0, 255, 186 },{ 0, 255, 190 },{ 0, 255, 194 },{ 0, 255, 198 },{ 0, 255, 201 },
{ 0, 255, 205 },{ 0, 255, 209 },{ 0, 255, 213 },{ 0, 255, 217 },{ 0, 255, 221 },
{ 0, 255, 224 },{ 0, 255, 228 },{ 0, 255, 232 },{ 0, 255, 236 },{ 0, 255, 240 },
{ 0, 255, 244 },{ 0, 255, 247 },{ 0, 255, 251 },{ 0, 255, 255 },{ 0, 251, 255 },
{ 0, 247, 255 },{ 0, 244, 255 },{ 0, 240, 255 },{ 0, 236, 255 },{ 0, 232, 255 },
{ 0, 228, 255 },{ 0, 224, 255 },{ 0, 221, 255 },{ 0, 217, 255 },{ 0, 213, 255 },
{ 0, 209, 255 },{ 0, 205, 255 },{ 0, 201, 255 },{ 0, 198, 255 },{ 0, 194, 255 },
{ 0, 190, 255 },{ 0, 186, 255 },{ 0, 182, 255 },{ 0, 178, 255 },{ 0, 175, 255 },
{ 0, 171, 255 },{ 0, 167, 255 },{ 0, 163, 255 },{ 0, 159, 255 },{ 0, 155, 255 },
{ 0, 152, 255 },{ 0, 148, 255 },{ 0, 144, 255 },{ 0, 140, 255 },{ 0, 136, 255 },
{ 0, 132, 255 },{ 0, 129, 255 },{ 0, 125, 255 },{ 0, 121, 255 },{ 0, 117, 255 },
{ 0, 113, 255 },{ 0, 110, 255 },{ 0, 106, 255 },{ 0, 102, 255 },{ 0,  98, 255 },
{ 0,  94, 255 },{ 0,  90, 255 },{ 0,  87, 255 },{ 0,  83, 255 },{ 0,  79, 255 },
{ 0,  75, 255 },{ 0,  71, 255 },{ 0,  67, 255 },{ 0,  64, 255 },{ 0,  60, 255 },
{ 0,  56, 255 },{ 0,  52, 255 },{ 0,  48, 255 },{ 0,  44, 255 },{ 0,  41, 255 },
{ 0,  37, 255 },{ 0,  33, 255 },{ 0,  29, 255 },{ 0,  25, 255 },{ 0,  21, 255 },
{ 0,  18, 255 },{ 0,  14, 255 },{ 0,  10, 255 },{ 0,   6, 255 },{ 0,   2, 255 },
{ 2,   0, 255 },{ 5,   0, 255 },{ 9,   0, 255 },{ 13,   0, 255 },{ 17,   0, 255 },
{ 21,   0, 255 },{ 25,   0, 255 },{ 28,   0, 255 },{ 32,   0, 255 },{ 36,   0, 255 },
{ 40,   0, 255 },{ 44,   0, 255 },{ 47,   0, 255 },{ 51,   0, 255 },{ 55,   0, 255 },
{ 59,   0, 255 },{ 63,   0, 255 },{ 67,   0, 255 },{ 70,   0, 255 },{ 74,   0, 255 },
{ 78,   0, 255 },{ 82,   0, 255 },{ 86,   0, 255 },{ 90,   0, 255 },{ 93,   0, 255 },
{ 97,   0, 255 },{ 101,   0, 255 },{ 105,   0, 255 },{ 109,   0, 255 },{ 113,   0, 255 },
{ 116,   0, 255 },{ 120,   0, 255 },{ 124,   0, 255 },{ 128,   0, 255 },{ 132,   0, 255 },
{ 136,   0, 255 },{ 139,   0, 255 },{ 143,   0, 255 },{ 147,   0, 255 },{ 151,   0, 255 },
{ 155,   0, 255 },{ 159,   0, 255 },{ 162,   0, 255 },{ 166,   0, 255 },{ 170,   0, 255 },
{ 174,   0, 255 },{ 178,   0, 255 },{ 181,   0, 255 },{ 185,   0, 255 },{ 189,   0, 255 },
{ 193,   0, 255 },{ 197,   0, 255 },{ 201,   0, 255 },{ 204,   0, 255 },{ 208,   0, 255 },
{ 212,   0, 255 },{ 216,   0, 255 },{ 220,   0, 255 },{ 224,   0, 255 },{ 227,   0, 255 },
{ 231,   0, 255 },{ 235,   0, 255 },{ 239,   0, 255 },{ 243,   0, 255 },{ 247,   0, 255 },
{ 250,   0, 255 },{ 254,   0, 255 },{ 252,   0, 252 },{ 248,   0, 248 },{ 244,   0, 244 },
{ 240,   0, 240 },{ 237,   0, 237 },{ 233,   0, 233 },{ 229,   0, 229 },{ 225,   0, 225 },
{ 221,   0, 221 },{ 217,   0, 217 },{ 214,   0, 214 },{ 210,   0, 210 },{ 206,   0, 206 },
{ 202,   0, 202 },{ 198,   0, 198 },{ 195,   0, 195 },{ 191,   0, 191 },{ 187,   0, 187 },
{ 183,   0, 183 },{ 179,   0, 179 },{ 175,   0, 175 },{ 172,   0, 172 },{ 168,   0, 168 },
{ 164,   0, 164 },{ 160,   0, 160 },{ 156,   0, 156 },{ 152,   0, 152 },{ 149,   0, 149 },
{ 145,   0, 145 },{ 141,   0, 141 },{ 137,   0, 137 },{ 133,   0, 133 },{ 129,   0, 129 },
{ 126,   0, 126 },{ 122,   0, 122 },{ 118,   0, 118 },{ 114,   0, 114 },{ 110,   0, 110 },
{ 106,   0, 106 },{ 103,   0, 103 },{ 99,   0,  99 },{ 95,   0,  95 },{ 91,   0,  91 },
{ 87,   0,  87 },{ 83,   0,  83 },{ 80,   0,  80 },{ 76,   0,  76 },{ 72,   0,  72 },
{ 68,   0,  68 },{ 64,   0,  64 },{ 60,   0,  60 },{ 57,   0,  57 },{ 53,   0,  53 },
{ 49,   0,  49 },{ 45,   0,  45 },{ 41,   0,  41 },{ 38,   0,  38 },{ 34,   0,  34 },
{ 30,   0,  30 },{ 26,   0,  26 },{ 22,   0,  22 },{ 18,   0,  18 },{ 15,   0,  15 },
{ 11,   0,  11 },{ 7,   0,   7 },{ 3,   0,   3 },{ 0,   0,   0 },{ 2,   2,   2 },
{ 4,   4,   4 },{ 6,   6,   6 },{ 8,   8,   8 },{ 10,  10,  10 },{ 12,  12,  12 },
{ 14,  14,  14 },{ 16,  16,  16 },{ 18,  18,  18 },{ 20,  20,  20 },{ 21,  21,  21 },
{ 23,  23,  23 },{ 25,  25,  25 },{ 27,  27,  27 },{ 29,  29,  29 },{ 31,  31,  31 },
{ 33,  33,  33 },{ 35,  35,  35 },{ 37,  37,  37 },{ 39,  39,  39 },{ 41,  41,  41 },
{ 43,  43,  43 },{ 44,  44,  44 },{ 46,  46,  46 },{ 48,  48,  48 },{ 50,  50,  50 },
{ 52,  52,  52 },{ 54,  54,  54 },{ 56,  56,  56 },{ 58,  58,  58 },{ 60,  60,  60 },
{ 62,  62,  62 },{ 64,  64,  64 },{ 65,  65,  65 },{ 67,  67,  67 },{ 69,  69,  69 },
{ 71,  71,  71 },{ 73,  73,  73 },{ 75,  75,  75 },{ 77,  77,  77 },{ 79,  79,  79 },
{ 81,  81,  81 },{ 83,  83,  83 },{ 85,  85,  85 },{ 87,  87,  87 },{ 88,  88,  88 },
{ 90,  90,  90 },{ 92,  92,  92 },{ 94,  94,  94 },{ 96,  96,  96 },{ 98,  98,  98 },
{ 100, 100, 100 },{ 102, 102, 102 },{ 104, 104, 104 },{ 106, 106, 106 },{ 108, 108, 108 },
{ 110, 110, 110 },{ 111, 111, 111 },{ 113, 113, 113 },{ 115, 115, 115 },{ 117, 117, 117 },
{ 119, 119, 119 },{ 121, 121, 121 },{ 123, 123, 123 },{ 125, 125, 125 },{ 127, 127, 127 },
{ 128, 126, 126 },{ 128, 124, 124 },{ 128, 123, 123 },{ 128, 121, 121 },{ 128, 119, 119 },
{ 128, 117, 117 },{ 128, 115, 115 },{ 128, 113, 113 },{ 128, 111, 111 },{ 128, 109, 109 },
{ 128, 107, 107 },{ 128, 105, 105 },{ 128, 103, 103 },{ 128, 101, 101 },{ 128, 100, 100 },
{ 128,  98,  98 },{ 128,  96,  96 },{ 128,  94,  94 },{ 128,  92,  92 },{ 128,  90,  90 },
{ 128,  88,  88 },{ 128,  86,  86 },{ 128,  84,  84 },{ 128,  82,  82 },{ 128,  80,  80 },
{ 128,  78,  78 },{ 128,  77,  77 },{ 128,  75,  75 },{ 128,  73,  73 },{ 128,  71,  71 },
{ 128,  69,  69 },{ 128,  67,  67 },{ 128,  65,  65 },{ 128,  63,  63 },{ 128,  61,  61 },
{ 128,  59,  59 },{ 128,  57,  57 },{ 128,  56,  56 },{ 128,  54,  54 },{ 128,  52,  52 },
{ 128,  50,  50 },{ 128,  48,  48 },{ 128,  46,  46 },{ 128,  44,  44 },{ 128,  42,  42 },
{ 128,  40,  40 },{ 128,  38,  38 },{ 128,  36,  36 },{ 128,  34,  34 },{ 128,  33,  33 },
{ 128,  31,  31 },{ 128,  29,  29 },{ 128,  27,  27 },{ 128,  25,  25 },{ 128,  23,  23 },
{ 128,  21,  21 },{ 128,  19,  19 },{ 128,  17,  17 },{ 128,  15,  15 },{ 128,  13,  13 },
{ 128,  11,  11 },{ 128,  10,  10 },{ 128,   8,   8 },{ 128,   6,   6 },{ 128,   4,   4 },
{ 128,   2,   2 },{ 128,   0,   0 },{ 128,   2,   0 },{ 128,   4,   0 },{ 128,   6,   0 },
{ 128,   8,   0 },{ 128,  10,   0 },{ 128,  11,   0 },{ 128,  13,   0 },{ 128,  15,   0 },
{ 128,  17,   0 },{ 128,  19,   0 },{ 128,  21,   0 },{ 128,  23,   0 },{ 128,  25,   0 },
{ 128,  27,   0 },{ 128,  29,   0 },{ 128,  31,   0 },{ 128,  33,   0 },{ 128,  34,   0 },
{ 128,  36,   0 },{ 128,  38,   0 },{ 128,  40,   0 },{ 128,  42,   0 },{ 128,  44,   0 },
{ 128,  46,   0 },{ 128,  48,   0 },{ 128,  50,   0 },{ 128,  52,   0 },{ 128,  54,   0 },
{ 128,  56,   0 },{ 128,  57,   0 },{ 128,  59,   0 },{ 128,  61,   0 },{ 128,  63,   0 },
{ 128,  65,   0 },{ 128,  67,   0 },{ 128,  69,   0 },{ 128,  71,   0 },{ 128,  73,   0 },
{ 128,  75,   0 },{ 128,  77,   0 },{ 128,  78,   0 },{ 128,  80,   0 },{ 128,  82,   0 },
{ 128,  84,   0 },{ 128,  86,   0 },{ 128,  88,   0 },{ 128,  90,   0 },{ 128,  92,   0 },
{ 128,  94,   0 },{ 128,  96,   0 },{ 128,  98,   0 },{ 128, 100,   0 },{ 128, 101,   0 },
{ 128, 103,   0 },{ 128, 105,   0 },{ 128, 107,   0 },{ 128, 109,   0 },{ 128, 111,   0 },
{ 128, 113,   0 },{ 128, 115,   0 },{ 128, 117,   0 },{ 128, 119,   0 },{ 128, 121,   0 },
{ 128, 123,   0 },{ 128, 124,   0 },{ 128, 126,   0 },{ 127, 128,   0 },{ 125, 128,   0 },
{ 123, 128,   0 },{ 121, 128,   0 },{ 119, 128,   0 },{ 117, 128,   0 },{ 115, 128,   0 },
{ 113, 128,   0 },{ 111, 128,   0 },{ 110, 128,   0 },{ 108, 128,   0 },{ 106, 128,   0 },
{ 104, 128,   0 },{ 102, 128,   0 },{ 100, 128,   0 },{ 98, 128,   0 },{ 96, 128,   0 },
{ 94, 128,   0 },{ 92, 128,   0 },{ 90, 128,   0 },{ 88, 128,   0 },{ 87, 128,   0 },
{ 85, 128,   0 },{ 83, 128,   0 },{ 81, 128,   0 },{ 79, 128,   0 },{ 77, 128,   0 },
{ 75, 128,   0 },{ 73, 128,   0 },{ 71, 128,   0 },{ 69, 128,   0 },{ 67, 128,   0 },
{ 65, 128,   0 },{ 64, 128,   0 },{ 62, 128,   0 },{ 60, 128,   0 },{ 58, 128,   0 },
{ 56, 128,   0 },{ 54, 128,   0 },{ 52, 128,   0 },{ 50, 128,   0 },{ 48, 128,   0 },
{ 46, 128,   0 },{ 44, 128,   0 },{ 43, 128,   0 },{ 41, 128,   0 },{ 39, 128,   0 },
{ 37, 128,   0 },{ 35, 128,   0 },{ 33, 128,   0 },{ 31, 128,   0 },{ 29, 128,   0 },
{ 27, 128,   0 },{ 25, 128,   0 },{ 23, 128,   0 },{ 21, 128,   0 },{ 20, 128,   0 },
{ 18, 128,   0 },{ 16, 128,   0 },{ 14, 128,   0 },{ 12, 128,   0 },{ 10, 128,   0 },
{ 8, 128,   0 },{ 6, 128,   0 },{ 4, 128,   0 },{ 2, 128,   0 },{ 0, 128,   0 },
{ 0, 128,   2 },{ 0, 128,   3 },{ 0, 128,   5 },{ 0, 128,   7 },{ 0, 128,   9 },
{ 0, 128,  11 },{ 0, 128,  13 },{ 0, 128,  15 },{ 0, 128,  17 },{ 0, 128,  19 },
{ 0, 128,  21 },{ 0, 128,  23 },{ 0, 128,  25 },{ 0, 128,  26 },{ 0, 128,  28 },
{ 0, 128,  30 },{ 0, 128,  32 },{ 0, 128,  34 },{ 0, 128,  36 },{ 0, 128,  38 },
{ 0, 128,  40 },{ 0, 128,  42 },{ 0, 128,  44 },{ 0, 128,  46 },{ 0, 128,  47 },
{ 0, 128,  49 },{ 0, 128,  51 },{ 0, 128,  53 },{ 0, 128,  55 },{ 0, 128,  57 },
{ 0, 128,  59 },{ 0, 128,  61 },{ 0, 128,  63 },{ 0, 128,  65 },{ 0, 128,  67 },
{ 0, 128,  69 },{ 0, 128,  70 },{ 0, 128,  72 },{ 0, 128,  74 },{ 0, 128,  76 },
{ 0, 128,  78 },{ 0, 128,  80 },{ 0, 128,  82 },{ 0, 128,  84 },{ 0, 128,  86 },
{ 0, 128,  88 },{ 0, 128,  90 },{ 0, 128,  92 },{ 0, 128,  93 },{ 0, 128,  95 },
{ 0, 128,  97 },{ 0, 128,  99 },{ 0, 128, 101 },{ 0, 128, 103 },{ 0, 128, 105 },
{ 0, 128, 107 },{ 0, 128, 109 },{ 0, 128, 111 },{ 0, 128, 113 },{ 0, 128, 114 },
{ 0, 128, 116 },{ 0, 128, 118 },{ 0, 128, 120 },{ 0, 128, 122 },{ 0, 128, 124 },
{ 0, 128, 126 },{ 0, 127, 128 },{ 0, 125, 128 },{ 0, 123, 128 },{ 0, 121, 128 },
{ 0, 119, 128 },{ 0, 118, 128 },{ 0, 116, 128 },{ 0, 114, 128 },{ 0, 112, 128 },
{ 0, 110, 128 },{ 0, 108, 128 },{ 0, 106, 128 },{ 0, 104, 128 },{ 0, 102, 128 },
{ 0, 100, 128 },{ 0,  98, 128 },{ 0,  96, 128 },{ 0,  95, 128 },{ 0,  93, 128 },
{ 0,  91, 128 },{ 0,  89, 128 },{ 0,  87, 128 },{ 0,  85, 128 },{ 0,  83, 128 },
{ 0,  81, 128 },{ 0,  79, 128 },{ 0,  77, 128 },{ 0,  75, 128 },{ 0,  74, 128 },
{ 0,  72, 128 },{ 0,  70, 128 },{ 0,  68, 128 },{ 0,  66, 128 },{ 0,  64, 128 },
{ 0,  62, 128 },{ 0,  60, 128 },{ 0,  58, 128 },{ 0,  56, 128 },{ 0,  54, 128 },
{ 0,  52, 128 },{ 0,  51, 128 },{ 0,  49, 128 },{ 0,  47, 128 },{ 0,  45, 128 },
{ 0,  43, 128 },{ 0,  41, 128 },{ 0,  39, 128 },{ 0,  37, 128 },{ 0,  35, 128 },
{ 0,  33, 128 },{ 0,  31, 128 },{ 0,  29, 128 },{ 0,  28, 128 },{ 0,  26, 128 },
{ 0,  24, 128 },{ 0,  22, 128 },{ 0,  20, 128 },{ 0,  18, 128 },{ 0,  16, 128 },
{ 0,  14, 128 },{ 0,  12, 128 },{ 0,  10, 128 },{ 0,   8, 128 },{ 0,   7, 128 },
{ 0,   5, 128 },{ 0,   3, 128 },{ 0,   1, 128 },{ 1,   0, 128 },{ 3,   0, 128 },
{ 5,   0, 128 },{ 7,   0, 128 },{ 9,   0, 128 },{ 11,   0, 128 },{ 13,   0, 128 },
{ 15,   0, 128 },{ 16,   0, 128 },{ 18,   0, 128 },{ 20,   0, 128 },{ 22,   0, 128 },
{ 24,   0, 128 },{ 26,   0, 128 },{ 28,   0, 128 },{ 30,   0, 128 },{ 32,   0, 128 },
{ 34,   0, 128 },{ 36,   0, 128 },{ 38,   0, 128 },{ 39,   0, 128 },{ 41,   0, 128 },
{ 43,   0, 128 },{ 45,   0, 128 },{ 47,   0, 128 },{ 49,   0, 128 },{ 51,   0, 128 },
{ 53,   0, 128 },{ 55,   0, 128 },{ 57,   0, 128 },{ 59,   0, 128 },{ 60,   0, 128 },
{ 62,   0, 128 },{ 64,   0, 128 },{ 66,   0, 128 },{ 68,   0, 128 },{ 70,   0, 128 },
{ 72,   0, 128 },{ 74,   0, 128 },{ 76,   0, 128 },{ 78,   0, 128 },{ 80,   0, 128 },
{ 82,   0, 128 },{ 83,   0, 128 },{ 85,   0, 128 },{ 87,   0, 128 },{ 89,   0, 128 },
{ 91,   0, 128 },{ 93,   0, 128 },{ 95,   0, 128 },{ 97,   0, 128 },{ 99,   0, 128 },
{ 101,   0, 128 },{ 103,   0, 128 },{ 105,   0, 128 },{ 106,   0, 128 },{ 108,   0, 128 },
{ 110,   0, 128 },{ 112,   0, 128 },{ 114,   0, 128 },{ 116,   0, 128 },{ 118,   0, 128 },
{ 120,   0, 128 },{ 122,   0, 128 },{ 124,   0, 128 },{ 126,   0, 128 },{ 128,   0, 128 }
};

const rgb_t yarg_colormap[1000] = {
    { 0,   0,   0 },{ 0,   0,   0 },{ 1,   1,   1 },{ 1,   1,   1 },{ 1,   1,   1 },
{ 1,   1,   1 },{ 2,   2,   2 },{ 2,   2,   2 },{ 2,   2,   2 },{ 2,   2,   2 },
{ 3,   3,   3 },{ 3,   3,   3 },{ 3,   3,   3 },{ 3,   3,   3 },{ 4,   4,   4 },
{ 4,   4,   4 },{ 4,   4,   4 },{ 4,   4,   4 },{ 5,   5,   5 },{ 5,   5,   5 },
{ 5,   5,   5 },{ 5,   5,   5 },{ 6,   6,   6 },{ 6,   6,   6 },{ 6,   6,   6 },
{ 6,   6,   6 },{ 7,   7,   7 },{ 7,   7,   7 },{ 7,   7,   7 },{ 7,   7,   7 },
{ 8,   8,   8 },{ 8,   8,   8 },{ 8,   8,   8 },{ 8,   8,   8 },{ 9,   9,   9 },
{ 9,   9,   9 },{ 9,   9,   9 },{ 9,   9,   9 },{ 10,  10,  10 },{ 10,  10,  10 },
{ 10,  10,  10 },{ 10,  10,  10 },{ 11,  11,  11 },{ 11,  11,  11 },{ 11,  11,  11 },
{ 11,  11,  11 },{ 12,  12,  12 },{ 12,  12,  12 },{ 12,  12,  12 },{ 13,  13,  13 },
{ 13,  13,  13 },{ 13,  13,  13 },{ 13,  13,  13 },{ 14,  14,  14 },{ 14,  14,  14 },
{ 14,  14,  14 },{ 14,  14,  14 },{ 15,  15,  15 },{ 15,  15,  15 },{ 15,  15,  15 },
{ 15,  15,  15 },{ 16,  16,  16 },{ 16,  16,  16 },{ 16,  16,  16 },{ 16,  16,  16 },
{ 17,  17,  17 },{ 17,  17,  17 },{ 17,  17,  17 },{ 17,  17,  17 },{ 18,  18,  18 },
{ 18,  18,  18 },{ 18,  18,  18 },{ 18,  18,  18 },{ 19,  19,  19 },{ 19,  19,  19 },
{ 19,  19,  19 },{ 19,  19,  19 },{ 20,  20,  20 },{ 20,  20,  20 },{ 20,  20,  20 },
{ 20,  20,  20 },{ 21,  21,  21 },{ 21,  21,  21 },{ 21,  21,  21 },{ 21,  21,  21 },
{ 22,  22,  22 },{ 22,  22,  22 },{ 22,  22,  22 },{ 22,  22,  22 },{ 23,  23,  23 },
{ 23,  23,  23 },{ 23,  23,  23 },{ 23,  23,  23 },{ 24,  24,  24 },{ 24,  24,  24 },
{ 24,  24,  24 },{ 25,  25,  25 },{ 25,  25,  25 },{ 25,  25,  25 },{ 25,  25,  25 },
{ 26,  26,  26 },{ 26,  26,  26 },{ 26,  26,  26 },{ 26,  26,  26 },{ 27,  27,  27 },
{ 27,  27,  27 },{ 27,  27,  27 },{ 27,  27,  27 },{ 28,  28,  28 },{ 28,  28,  28 },
{ 28,  28,  28 },{ 28,  28,  28 },{ 29,  29,  29 },{ 29,  29,  29 },{ 29,  29,  29 },
{ 29,  29,  29 },{ 30,  30,  30 },{ 30,  30,  30 },{ 30,  30,  30 },{ 30,  30,  30 },
{ 31,  31,  31 },{ 31,  31,  31 },{ 31,  31,  31 },{ 31,  31,  31 },{ 32,  32,  32 },
{ 32,  32,  32 },{ 32,  32,  32 },{ 32,  32,  32 },{ 33,  33,  33 },{ 33,  33,  33 },
{ 33,  33,  33 },{ 33,  33,  33 },{ 34,  34,  34 },{ 34,  34,  34 },{ 34,  34,  34 },
{ 34,  34,  34 },{ 35,  35,  35 },{ 35,  35,  35 },{ 35,  35,  35 },{ 35,  35,  35 },
{ 36,  36,  36 },{ 36,  36,  36 },{ 36,  36,  36 },{ 37,  37,  37 },{ 37,  37,  37 },
{ 37,  37,  37 },{ 37,  37,  37 },{ 38,  38,  38 },{ 38,  38,  38 },{ 38,  38,  38 },
{ 38,  38,  38 },{ 39,  39,  39 },{ 39,  39,  39 },{ 39,  39,  39 },{ 39,  39,  39 },
{ 40,  40,  40 },{ 40,  40,  40 },{ 40,  40,  40 },{ 40,  40,  40 },{ 41,  41,  41 },
{ 41,  41,  41 },{ 41,  41,  41 },{ 41,  41,  41 },{ 42,  42,  42 },{ 42,  42,  42 },
{ 42,  42,  42 },{ 42,  42,  42 },{ 43,  43,  43 },{ 43,  43,  43 },{ 43,  43,  43 },
{ 43,  43,  43 },{ 44,  44,  44 },{ 44,  44,  44 },{ 44,  44,  44 },{ 44,  44,  44 },
{ 45,  45,  45 },{ 45,  45,  45 },{ 45,  45,  45 },{ 45,  45,  45 },{ 46,  46,  46 },
{ 46,  46,  46 },{ 46,  46,  46 },{ 46,  46,  46 },{ 47,  47,  47 },{ 47,  47,  47 },
{ 47,  47,  47 },{ 47,  47,  47 },{ 48,  48,  48 },{ 48,  48,  48 },{ 48,  48,  48 },
{ 48,  48,  48 },{ 49,  49,  49 },{ 49,  49,  49 },{ 49,  49,  49 },{ 50,  50,  50 },
{ 50,  50,  50 },{ 50,  50,  50 },{ 50,  50,  50 },{ 51,  51,  51 },{ 51,  51,  51 },
{ 51,  51,  51 },{ 51,  51,  51 },{ 52,  52,  52 },{ 52,  52,  52 },{ 52,  52,  52 },
{ 52,  52,  52 },{ 53,  53,  53 },{ 53,  53,  53 },{ 53,  53,  53 },{ 53,  53,  53 },
{ 54,  54,  54 },{ 54,  54,  54 },{ 54,  54,  54 },{ 54,  54,  54 },{ 55,  55,  55 },
{ 55,  55,  55 },{ 55,  55,  55 },{ 55,  55,  55 },{ 56,  56,  56 },{ 56,  56,  56 },
{ 56,  56,  56 },{ 56,  56,  56 },{ 57,  57,  57 },{ 57,  57,  57 },{ 57,  57,  57 },
{ 57,  57,  57 },{ 58,  58,  58 },{ 58,  58,  58 },{ 58,  58,  58 },{ 58,  58,  58 },
{ 59,  59,  59 },{ 59,  59,  59 },{ 59,  59,  59 },{ 59,  59,  59 },{ 60,  60,  60 },
{ 60,  60,  60 },{ 60,  60,  60 },{ 60,  60,  60 },{ 61,  61,  61 },{ 61,  61,  61 },
{ 61,  61,  61 },{ 62,  62,  62 },{ 62,  62,  62 },{ 62,  62,  62 },{ 62,  62,  62 },
{ 63,  63,  63 },{ 63,  63,  63 },{ 63,  63,  63 },{ 63,  63,  63 },{ 64,  64,  64 },
{ 64,  64,  64 },{ 64,  64,  64 },{ 64,  64,  64 },{ 65,  65,  65 },{ 65,  65,  65 },
{ 65,  65,  65 },{ 65,  65,  65 },{ 66,  66,  66 },{ 66,  66,  66 },{ 66,  66,  66 },
{ 66,  66,  66 },{ 67,  67,  67 },{ 67,  67,  67 },{ 67,  67,  67 },{ 67,  67,  67 },
{ 68,  68,  68 },{ 68,  68,  68 },{ 68,  68,  68 },{ 68,  68,  68 },{ 69,  69,  69 },
{ 69,  69,  69 },{ 69,  69,  69 },{ 69,  69,  69 },{ 70,  70,  70 },{ 70,  70,  70 },
{ 70,  70,  70 },{ 70,  70,  70 },{ 71,  71,  71 },{ 71,  71,  71 },{ 71,  71,  71 },
{ 71,  71,  71 },{ 72,  72,  72 },{ 72,  72,  72 },{ 72,  72,  72 },{ 72,  72,  72 },
{ 73,  73,  73 },{ 73,  73,  73 },{ 73,  73,  73 },{ 74,  74,  74 },{ 74,  74,  74 },
{ 74,  74,  74 },{ 74,  74,  74 },{ 75,  75,  75 },{ 75,  75,  75 },{ 75,  75,  75 },
{ 75,  75,  75 },{ 76,  76,  76 },{ 76,  76,  76 },{ 76,  76,  76 },{ 76,  76,  76 },
{ 77,  77,  77 },{ 77,  77,  77 },{ 77,  77,  77 },{ 77,  77,  77 },{ 78,  78,  78 },
{ 78,  78,  78 },{ 78,  78,  78 },{ 78,  78,  78 },{ 79,  79,  79 },{ 79,  79,  79 },
{ 79,  79,  79 },{ 79,  79,  79 },{ 80,  80,  80 },{ 80,  80,  80 },{ 80,  80,  80 },
{ 80,  80,  80 },{ 81,  81,  81 },{ 81,  81,  81 },{ 81,  81,  81 },{ 81,  81,  81 },
{ 82,  82,  82 },{ 82,  82,  82 },{ 82,  82,  82 },{ 82,  82,  82 },{ 83,  83,  83 },
{ 83,  83,  83 },{ 83,  83,  83 },{ 83,  83,  83 },{ 84,  84,  84 },{ 84,  84,  84 },
{ 84,  84,  84 },{ 84,  84,  84 },{ 85,  85,  85 },{ 85,  85,  85 },{ 85,  85,  85 },
{ 86,  86,  86 },{ 86,  86,  86 },{ 86,  86,  86 },{ 86,  86,  86 },{ 87,  87,  87 },
{ 87,  87,  87 },{ 87,  87,  87 },{ 87,  87,  87 },{ 88,  88,  88 },{ 88,  88,  88 },
{ 88,  88,  88 },{ 88,  88,  88 },{ 89,  89,  89 },{ 89,  89,  89 },{ 89,  89,  89 },
{ 89,  89,  89 },{ 90,  90,  90 },{ 90,  90,  90 },{ 90,  90,  90 },{ 90,  90,  90 },
{ 91,  91,  91 },{ 91,  91,  91 },{ 91,  91,  91 },{ 91,  91,  91 },{ 92,  92,  92 },
{ 92,  92,  92 },{ 92,  92,  92 },{ 92,  92,  92 },{ 93,  93,  93 },{ 93,  93,  93 },
{ 93,  93,  93 },{ 93,  93,  93 },{ 94,  94,  94 },{ 94,  94,  94 },{ 94,  94,  94 },
{ 94,  94,  94 },{ 95,  95,  95 },{ 95,  95,  95 },{ 95,  95,  95 },{ 95,  95,  95 },
{ 96,  96,  96 },{ 96,  96,  96 },{ 96,  96,  96 },{ 96,  96,  96 },{ 97,  97,  97 },
{ 97,  97,  97 },{ 97,  97,  97 },{ 98,  98,  98 },{ 98,  98,  98 },{ 98,  98,  98 },
{ 98,  98,  98 },{ 99,  99,  99 },{ 99,  99,  99 },{ 99,  99,  99 },{ 99,  99,  99 },
{ 100, 100, 100 },{ 100, 100, 100 },{ 100, 100, 100 },{ 100, 100, 100 },{ 101, 101, 101 },
{ 101, 101, 101 },{ 101, 101, 101 },{ 101, 101, 101 },{ 102, 102, 102 },{ 102, 102, 102 },
{ 102, 102, 102 },{ 102, 102, 102 },{ 103, 103, 103 },{ 103, 103, 103 },{ 103, 103, 103 },
{ 103, 103, 103 },{ 104, 104, 104 },{ 104, 104, 104 },{ 104, 104, 104 },{ 104, 104, 104 },
{ 105, 105, 105 },{ 105, 105, 105 },{ 105, 105, 105 },{ 105, 105, 105 },{ 106, 106, 106 },
{ 106, 106, 106 },{ 106, 106, 106 },{ 106, 106, 106 },{ 107, 107, 107 },{ 107, 107, 107 },
{ 107, 107, 107 },{ 107, 107, 107 },{ 108, 108, 108 },{ 108, 108, 108 },{ 108, 108, 108 },
{ 108, 108, 108 },{ 109, 109, 109 },{ 109, 109, 109 },{ 109, 109, 109 },{ 110, 110, 110 },
{ 110, 110, 110 },{ 110, 110, 110 },{ 110, 110, 110 },{ 111, 111, 111 },{ 111, 111, 111 },
{ 111, 111, 111 },{ 111, 111, 111 },{ 112, 112, 112 },{ 112, 112, 112 },{ 112, 112, 112 },
{ 112, 112, 112 },{ 113, 113, 113 },{ 113, 113, 113 },{ 113, 113, 113 },{ 113, 113, 113 },
{ 114, 114, 114 },{ 114, 114, 114 },{ 114, 114, 114 },{ 114, 114, 114 },{ 115, 115, 115 },
{ 115, 115, 115 },{ 115, 115, 115 },{ 115, 115, 115 },{ 116, 116, 116 },{ 116, 116, 116 },
{ 116, 116, 116 },{ 116, 116, 116 },{ 117, 117, 117 },{ 117, 117, 117 },{ 117, 117, 117 },
{ 117, 117, 117 },{ 118, 118, 118 },{ 118, 118, 118 },{ 118, 118, 118 },{ 118, 118, 118 },
{ 119, 119, 119 },{ 119, 119, 119 },{ 119, 119, 119 },{ 119, 119, 119 },{ 120, 120, 120 },
{ 120, 120, 120 },{ 120, 120, 120 },{ 120, 120, 120 },{ 121, 121, 121 },{ 121, 121, 121 },
{ 121, 121, 121 },{ 122, 122, 122 },{ 122, 122, 122 },{ 122, 122, 122 },{ 122, 122, 122 },
{ 123, 123, 123 },{ 123, 123, 123 },{ 123, 123, 123 },{ 123, 123, 123 },{ 124, 124, 124 },
{ 124, 124, 124 },{ 124, 124, 124 },{ 124, 124, 124 },{ 125, 125, 125 },{ 125, 125, 125 },
{ 125, 125, 125 },{ 125, 125, 125 },{ 126, 126, 126 },{ 126, 126, 126 },{ 126, 126, 126 },
{ 126, 126, 126 },{ 127, 127, 127 },{ 127, 127, 127 },{ 127, 127, 127 },{ 127, 127, 127 },
{ 128, 128, 128 },{ 128, 128, 128 },{ 128, 128, 128 },{ 128, 128, 128 },{ 129, 129, 129 },
{ 129, 129, 129 },{ 129, 129, 129 },{ 129, 129, 129 },{ 130, 130, 130 },{ 130, 130, 130 },
{ 130, 130, 130 },{ 130, 130, 130 },{ 131, 131, 131 },{ 131, 131, 131 },{ 131, 131, 131 },
{ 131, 131, 131 },{ 132, 132, 132 },{ 132, 132, 132 },{ 132, 132, 132 },{ 132, 132, 132 },
{ 133, 133, 133 },{ 133, 133, 133 },{ 133, 133, 133 },{ 133, 133, 133 },{ 134, 134, 134 },
{ 134, 134, 134 },{ 134, 134, 134 },{ 135, 135, 135 },{ 135, 135, 135 },{ 135, 135, 135 },
{ 135, 135, 135 },{ 136, 136, 136 },{ 136, 136, 136 },{ 136, 136, 136 },{ 136, 136, 136 },
{ 137, 137, 137 },{ 137, 137, 137 },{ 137, 137, 137 },{ 137, 137, 137 },{ 138, 138, 138 },
{ 138, 138, 138 },{ 138, 138, 138 },{ 138, 138, 138 },{ 139, 139, 139 },{ 139, 139, 139 },
{ 139, 139, 139 },{ 139, 139, 139 },{ 140, 140, 140 },{ 140, 140, 140 },{ 140, 140, 140 },
{ 140, 140, 140 },{ 141, 141, 141 },{ 141, 141, 141 },{ 141, 141, 141 },{ 141, 141, 141 },
{ 142, 142, 142 },{ 142, 142, 142 },{ 142, 142, 142 },{ 142, 142, 142 },{ 143, 143, 143 },
{ 143, 143, 143 },{ 143, 143, 143 },{ 143, 143, 143 },{ 144, 144, 144 },{ 144, 144, 144 },
{ 144, 144, 144 },{ 144, 144, 144 },{ 145, 145, 145 },{ 145, 145, 145 },{ 145, 145, 145 },
{ 145, 145, 145 },{ 146, 146, 146 },{ 146, 146, 146 },{ 146, 146, 146 },{ 147, 147, 147 },
{ 147, 147, 147 },{ 147, 147, 147 },{ 147, 147, 147 },{ 148, 148, 148 },{ 148, 148, 148 },
{ 148, 148, 148 },{ 148, 148, 148 },{ 149, 149, 149 },{ 149, 149, 149 },{ 149, 149, 149 },
{ 149, 149, 149 },{ 150, 150, 150 },{ 150, 150, 150 },{ 150, 150, 150 },{ 150, 150, 150 },
{ 151, 151, 151 },{ 151, 151, 151 },{ 151, 151, 151 },{ 151, 151, 151 },{ 152, 152, 152 },
{ 152, 152, 152 },{ 152, 152, 152 },{ 152, 152, 152 },{ 153, 153, 153 },{ 153, 153, 153 },
{ 153, 153, 153 },{ 153, 153, 153 },{ 154, 154, 154 },{ 154, 154, 154 },{ 154, 154, 154 },
{ 154, 154, 154 },{ 155, 155, 155 },{ 155, 155, 155 },{ 155, 155, 155 },{ 155, 155, 155 },
{ 156, 156, 156 },{ 156, 156, 156 },{ 156, 156, 156 },{ 156, 156, 156 },{ 157, 157, 157 },
{ 157, 157, 157 },{ 157, 157, 157 },{ 157, 157, 157 },{ 158, 158, 158 },{ 158, 158, 158 },
{ 158, 158, 158 },{ 159, 159, 159 },{ 159, 159, 159 },{ 159, 159, 159 },{ 159, 159, 159 },
{ 160, 160, 160 },{ 160, 160, 160 },{ 160, 160, 160 },{ 160, 160, 160 },{ 161, 161, 161 },
{ 161, 161, 161 },{ 161, 161, 161 },{ 161, 161, 161 },{ 162, 162, 162 },{ 162, 162, 162 },
{ 162, 162, 162 },{ 162, 162, 162 },{ 163, 163, 163 },{ 163, 163, 163 },{ 163, 163, 163 },
{ 163, 163, 163 },{ 164, 164, 164 },{ 164, 164, 164 },{ 164, 164, 164 },{ 164, 164, 164 },
{ 165, 165, 165 },{ 165, 165, 165 },{ 165, 165, 165 },{ 165, 165, 165 },{ 166, 166, 166 },
{ 166, 166, 166 },{ 166, 166, 166 },{ 166, 166, 166 },{ 167, 167, 167 },{ 167, 167, 167 },
{ 167, 167, 167 },{ 167, 167, 167 },{ 168, 168, 168 },{ 168, 168, 168 },{ 168, 168, 168 },
{ 168, 168, 168 },{ 169, 169, 169 },{ 169, 169, 169 },{ 169, 169, 169 },{ 169, 169, 169 },
{ 170, 170, 170 },{ 170, 170, 170 },{ 170, 170, 170 },{ 171, 171, 171 },{ 171, 171, 171 },
{ 171, 171, 171 },{ 171, 171, 171 },{ 172, 172, 172 },{ 172, 172, 172 },{ 172, 172, 172 },
{ 172, 172, 172 },{ 173, 173, 173 },{ 173, 173, 173 },{ 173, 173, 173 },{ 173, 173, 173 },
{ 174, 174, 174 },{ 174, 174, 174 },{ 174, 174, 174 },{ 174, 174, 174 },{ 175, 175, 175 },
{ 175, 175, 175 },{ 175, 175, 175 },{ 175, 175, 175 },{ 176, 176, 176 },{ 176, 176, 176 },
{ 176, 176, 176 },{ 176, 176, 176 },{ 177, 177, 177 },{ 177, 177, 177 },{ 177, 177, 177 },
{ 177, 177, 177 },{ 178, 178, 178 },{ 178, 178, 178 },{ 178, 178, 178 },{ 178, 178, 178 },
{ 179, 179, 179 },{ 179, 179, 179 },{ 179, 179, 179 },{ 179, 179, 179 },{ 180, 180, 180 },
{ 180, 180, 180 },{ 180, 180, 180 },{ 180, 180, 180 },{ 181, 181, 181 },{ 181, 181, 181 },
{ 181, 181, 181 },{ 181, 181, 181 },{ 182, 182, 182 },{ 182, 182, 182 },{ 182, 182, 182 },
{ 183, 183, 183 },{ 183, 183, 183 },{ 183, 183, 183 },{ 183, 183, 183 },{ 184, 184, 184 },
{ 184, 184, 184 },{ 184, 184, 184 },{ 184, 184, 184 },{ 185, 185, 185 },{ 185, 185, 185 },
{ 185, 185, 185 },{ 185, 185, 185 },{ 186, 186, 186 },{ 186, 186, 186 },{ 186, 186, 186 },
{ 186, 186, 186 },{ 187, 187, 187 },{ 187, 187, 187 },{ 187, 187, 187 },{ 187, 187, 187 },
{ 188, 188, 188 },{ 188, 188, 188 },{ 188, 188, 188 },{ 188, 188, 188 },{ 189, 189, 189 },
{ 189, 189, 189 },{ 189, 189, 189 },{ 189, 189, 189 },{ 190, 190, 190 },{ 190, 190, 190 },
{ 190, 190, 190 },{ 190, 190, 190 },{ 191, 191, 191 },{ 191, 191, 191 },{ 191, 191, 191 },
{ 191, 191, 191 },{ 192, 192, 192 },{ 192, 192, 192 },{ 192, 192, 192 },{ 192, 192, 192 },
{ 193, 193, 193 },{ 193, 193, 193 },{ 193, 193, 193 },{ 193, 193, 193 },{ 194, 194, 194 },
{ 194, 194, 194 },{ 194, 194, 194 },{ 195, 195, 195 },{ 195, 195, 195 },{ 195, 195, 195 },
{ 195, 195, 195 },{ 196, 196, 196 },{ 196, 196, 196 },{ 196, 196, 196 },{ 196, 196, 196 },
{ 197, 197, 197 },{ 197, 197, 197 },{ 197, 197, 197 },{ 197, 197, 197 },{ 198, 198, 198 },
{ 198, 198, 198 },{ 198, 198, 198 },{ 198, 198, 198 },{ 199, 199, 199 },{ 199, 199, 199 },
{ 199, 199, 199 },{ 199, 199, 199 },{ 200, 200, 200 },{ 200, 200, 200 },{ 200, 200, 200 },
{ 200, 200, 200 },{ 201, 201, 201 },{ 201, 201, 201 },{ 201, 201, 201 },{ 201, 201, 201 },
{ 202, 202, 202 },{ 202, 202, 202 },{ 202, 202, 202 },{ 202, 202, 202 },{ 203, 203, 203 },
{ 203, 203, 203 },{ 203, 203, 203 },{ 203, 203, 203 },{ 204, 204, 204 },{ 204, 204, 204 },
{ 204, 204, 204 },{ 204, 204, 204 },{ 205, 205, 205 },{ 205, 205, 205 },{ 205, 205, 205 },
{ 205, 205, 205 },{ 206, 206, 206 },{ 206, 206, 206 },{ 206, 206, 206 },{ 207, 207, 207 },
{ 207, 207, 207 },{ 207, 207, 207 },{ 207, 207, 207 },{ 208, 208, 208 },{ 208, 208, 208 },
{ 208, 208, 208 },{ 208, 208, 208 },{ 209, 209, 209 },{ 209, 209, 209 },{ 209, 209, 209 },
{ 209, 209, 209 },{ 210, 210, 210 },{ 210, 210, 210 },{ 210, 210, 210 },{ 210, 210, 210 },
{ 211, 211, 211 },{ 211, 211, 211 },{ 211, 211, 211 },{ 211, 211, 211 },{ 212, 212, 212 },
{ 212, 212, 212 },{ 212, 212, 212 },{ 212, 212, 212 },{ 213, 213, 213 },{ 213, 213, 213 },
{ 213, 213, 213 },{ 213, 213, 213 },{ 214, 214, 214 },{ 214, 214, 214 },{ 214, 214, 214 },
{ 214, 214, 214 },{ 215, 215, 215 },{ 215, 215, 215 },{ 215, 215, 215 },{ 215, 215, 215 },
{ 216, 216, 216 },{ 216, 216, 216 },{ 216, 216, 216 },{ 216, 216, 216 },{ 217, 217, 217 },
{ 217, 217, 217 },{ 217, 217, 217 },{ 217, 217, 217 },{ 218, 218, 218 },{ 218, 218, 218 },
{ 218, 218, 218 },{ 218, 218, 218 },{ 219, 219, 219 },{ 219, 219, 219 },{ 219, 219, 219 },
{ 220, 220, 220 },{ 220, 220, 220 },{ 220, 220, 220 },{ 220, 220, 220 },{ 221, 221, 221 },
{ 221, 221, 221 },{ 221, 221, 221 },{ 221, 221, 221 },{ 222, 222, 222 },{ 222, 222, 222 },
{ 222, 222, 222 },{ 222, 222, 222 },{ 223, 223, 223 },{ 223, 223, 223 },{ 223, 223, 223 },
{ 223, 223, 223 },{ 224, 224, 224 },{ 224, 224, 224 },{ 224, 224, 224 },{ 224, 224, 224 },
{ 225, 225, 225 },{ 225, 225, 225 },{ 225, 225, 225 },{ 225, 225, 225 },{ 226, 226, 226 },
{ 226, 226, 226 },{ 226, 226, 226 },{ 226, 226, 226 },{ 227, 227, 227 },{ 227, 227, 227 },
{ 227, 227, 227 },{ 227, 227, 227 },{ 228, 228, 228 },{ 228, 228, 228 },{ 228, 228, 228 },
{ 228, 228, 228 },{ 229, 229, 229 },{ 229, 229, 229 },{ 229, 229, 229 },{ 229, 229, 229 },
{ 230, 230, 230 },{ 230, 230, 230 },{ 230, 230, 230 },{ 230, 230, 230 },{ 231, 231, 231 },
{ 231, 231, 231 },{ 231, 231, 231 },{ 232, 232, 232 },{ 232, 232, 232 },{ 232, 232, 232 },
{ 232, 232, 232 },{ 233, 233, 233 },{ 233, 233, 233 },{ 233, 233, 233 },{ 233, 233, 233 },
{ 234, 234, 234 },{ 234, 234, 234 },{ 234, 234, 234 },{ 234, 234, 234 },{ 235, 235, 235 },
{ 235, 235, 235 },{ 235, 235, 235 },{ 235, 235, 235 },{ 236, 236, 236 },{ 236, 236, 236 },
{ 236, 236, 236 },{ 236, 236, 236 },{ 237, 237, 237 },{ 237, 237, 237 },{ 237, 237, 237 },
{ 237, 237, 237 },{ 238, 238, 238 },{ 238, 238, 238 },{ 238, 238, 238 },{ 238, 238, 238 },
{ 239, 239, 239 },{ 239, 239, 239 },{ 239, 239, 239 },{ 239, 239, 239 },{ 240, 240, 240 },
{ 240, 240, 240 },{ 240, 240, 240 },{ 240, 240, 240 },{ 241, 241, 241 },{ 241, 241, 241 },
{ 241, 241, 241 },{ 241, 241, 241 },{ 242, 242, 242 },{ 242, 242, 242 },{ 242, 242, 242 },
{ 242, 242, 242 },{ 243, 243, 243 },{ 243, 243, 243 },{ 243, 243, 243 },{ 244, 244, 244 },
{ 244, 244, 244 },{ 244, 244, 244 },{ 244, 244, 244 },{ 245, 245, 245 },{ 245, 245, 245 },
{ 245, 245, 245 },{ 245, 245, 245 },{ 246, 246, 246 },{ 246, 246, 246 },{ 246, 246, 246 },
{ 246, 246, 246 },{ 247, 247, 247 },{ 247, 247, 247 },{ 247, 247, 247 },{ 247, 247, 247 },
{ 248, 248, 248 },{ 248, 248, 248 },{ 248, 248, 248 },{ 248, 248, 248 },{ 249, 249, 249 },
{ 249, 249, 249 },{ 249, 249, 249 },{ 249, 249, 249 },{ 250, 250, 250 },{ 250, 250, 250 },
{ 250, 250, 250 },{ 250, 250, 250 },{ 251, 251, 251 },{ 251, 251, 251 },{ 251, 251, 251 },
{ 251, 251, 251 },{ 252, 252, 252 },{ 252, 252, 252 },{ 252, 252, 252 },{ 252, 252, 252 },
{ 253, 253, 253 },{ 253, 253, 253 },{ 253, 253, 253 },{ 253, 253, 253 },{ 254, 254, 254 },
{ 254, 254, 254 },{ 254, 254, 254 },{ 254, 254, 254 },{ 255, 255, 255 },{ 255, 255, 255 }
};


}} //namespace
// clang-format on
#endif