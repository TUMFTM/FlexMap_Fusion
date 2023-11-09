// Copyright 2023 Maximilian Leitenstern
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ==========================================
// Author: Maximilian Leitenstern (TUM)
// Date: 23.03.2023
// ==========================================
//
//
#pragma once

#include <string>

/*****************************************************************
 * Specify RGB codes of TUMcolors to be used for visualization
 ******************************************************************/
struct s_TUMcolor
{
  explicit s_TUMcolor(const std::string name)
  {
    // Presentation
    if (name == "Blue") {
      this->r = 0.0 / 255.0;
      this->g = 101.0 / 255.0;
      this->b = 189.0 / 255.0;
    } else if (name == "Blue1") {
      this->r = 0.0 / 255.0;
      this->g = 51.0 / 255.0;
      this->b = 89.0 / 255.0;
    } else if (name == "Blue2") {
      this->r = 0.0 / 255.0;
      this->g = 82.0 / 255.0;
      this->b = 147.0 / 255.0;
    } else if (name == "Blue3") {
      this->r = 100.0 / 255.0;
      this->g = 160.0 / 255.0;
      this->b = 200.0 / 255.0;
    } else if (name == "Blue4") {
      this->r = 152.0 / 255.0;
      this->g = 198.0 / 255.0;
      this->b = 234.0 / 255.0;
    } else if (name == "Gray1") {
      this->r = 51.0 / 255.0;
      this->g = 51.0 / 255.0;
      this->b = 51.0 / 255.0;
    } else if (name == "Gray2") {
      this->r = 127.0 / 255.0;
      this->g = 127.0 / 255.0;
      this->b = 127.0 / 255.0;
    } else if (name == "Gray3") {
      this->r = 204.0 / 255.0;
      this->g = 204.0 / 255.0;
      this->b = 204.0 / 255.0;
    } else if (name == "Ivory") {
      this->r = 218.0 / 255.0;
      this->g = 215.0 / 255.0;
      this->b = 203.0 / 255.0;
    } else if (name == "Orange") {
      this->r = 227.0 / 255.0;
      this->g = 114.0 / 255.0;
      this->b = 34.0 / 255.0;
    } else if (name == "Green") {
      this->r = 162.0 / 255.0;
      this->g = 173.0 / 255.0;
      this->b = 0.0 / 255.0;
    } else if (name == "Black") {
      this->r = 0.0 / 255.0;
      this->g = 0.0 / 255.0;
      this->b = 0.0 / 255.0;
      // Web
    } else if (name == "WEBBlueDark") {
      this->r = 7.0 / 255.0;
      this->g = 33.0 / 255.0;
      this->b = 64.0 / 255.0;
    } else if (name == "WEBBlueLight") {
      this->r = 94.0 / 255.0;
      this->g = 148.0 / 255.0;
      this->b = 212.0 / 255.0;
    } else if (name == "WEBYellow") {
      this->r = 254.0 / 255.0;
      this->g = 215.0 / 255.0;
      this->b = 2.0 / 255.0;
    } else if (name == "WEBOrange") {
      this->r = 247.0 / 255.0;
      this->g = 129.0 / 255.0;
      this->b = 30.0 / 255.0;
    } else if (name == "WEBPink") {
      this->r = 181.0 / 255.0;
      this->g = 92.0 / 255.0;
      this->b = 165.0 / 255.0;
    } else if (name == "WEBBlueBright") {
      this->r = 143.0 / 255.0;
      this->g = 129.0 / 255.0;
      this->b = 234.0 / 255.0;
    } else if (name == "WEBRed") {
      this->r = 234.0 / 255.0;
      this->g = 114.0 / 255.0;
      this->b = 55.0 / 255.0;
    } else if (name == "WEBGreen") {
      this->r = 159.0 / 255.0;
      this->g = 186.0 / 255.0;
      this->b = 54.0 / 255.0;
      // otherwise white
    } else {
      this->r = 255.0 / 255.0;
      this->g = 255.0 / 255.0;
      this->b = 255.0 / 255.0;
    }
  }
  double r, g, b;
};
