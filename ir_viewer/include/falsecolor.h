/*
 *
 * Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the authors at <firstname.lastname at ethz dot ch>
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef FALSECOLOR_H_
#define FALSECOLOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/contrib/retina.hpp>
#include <boost/shared_ptr.hpp>

class converter_16_8
{
  enum
  {
    histminmembersperbucket = 10,
  };
private:
  double max_;
  double min_;
  static converter_16_8* inst_;
  bool firstframe_;
  boost::shared_ptr<cv::Retina> retina_;
public:
  converter_16_8();
  ~converter_16_8();
  static converter_16_8& Instance()
  {
    if (!inst_)
    {
      inst_ = new converter_16_8;
    }
    return *inst_;
  }
  double getMax();
  double getMin();
  void convert_to8bit(const cv::Mat& img16, cv::Mat& img8, bool doTempConversion);
  void toneMapping(const cv::Mat& img16, cv::Mat& img8);
};

struct color
{
  unsigned char rgbBlue;
  unsigned char rgbGreen;
  unsigned char rgbRed;
  color()
  {
    rgbBlue = rgbGreen = rgbRed = 0;
  }
};

struct palette
{
  enum palettetypes{
    Linear_red_palettes, GammaLog_red_palettes, Inversion_red_palette, Linear_palettes, GammaLog_palettes,
    Inversion_palette, False_color_palette1, False_color_palette2, False_color_palette3, False_color_palette4
  };
  color colors[256];
};

palette GetPalette(palette::palettetypes pal);

void convertFalseColor(const cv::Mat& srcmat, cv::Mat& dstmat, palette::palettetypes paltype, bool drawlegend = false, double mintemp = 0, double maxtemp = 0);

#endif /* FALSECOLOR_H_ */
