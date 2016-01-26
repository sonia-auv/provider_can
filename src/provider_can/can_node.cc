/**
 * \file	can_node.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	26/01/2016
 *
 * \copyright Copyright (c) 2015 Copyright
 *
 * \section LICENSE http://www.gnu.org/licenses/gpl-3.0.en.html
 *
 * Changes by: S.O.N.I.A.
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "can_node.h"

namespace provider_can {

//==============================================================================
// C / D T O R   S E C T I O N

  CanNode::CanNode() {
    can_ptr = std::make_shared<provider_can::CanDispatcher>
      (controllers,on_board_pc,0, BAUD_125K, 10);

    bottom_light_ = std::make_shared<provider_can::BottomLight>(can_ptr);
  }
//------------------------------------------------------------------------------
//

  CanNode::~CanNode() {

  }

//==============================================================================
// M E T H O D S   S E C T I O N

  void CanNode::ProcessMessages(void) {
    bottom_light_->Process();
  }


}