/**
 * \file	can_driver.h
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	25/11/2015
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

#ifndef PROVIDER_CAN_CAN_DRIVER_H_
#define PROVIDER_CAN_CAN_DRIVER_H_


namespace provider_can {

    class CanDriver {
        //============================================================================
        // T Y P E D E F   A N D   E N U M


    public:
        //============================================================================
        // P U B L I C   C / D T O R S

        //! Constructor
        CanDriver();

        // Destructor
        ~CanDriver();

        //============================================================================
        // P U B L I C   M E T H O D S

        //============================================================================
        // P U B L I C   M E M B E R S

    private:
        //============================================================================
        // P R I V A T E   M E T H O D S

        //============================================================================
        // P R I V A T E   M E M B E R S
    };

}  // namespace provider_can

#endif  // PROVIDER_CAN_CAN_DRIVER_H_
