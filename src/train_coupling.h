/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file train_coupling.h Helpers for selecting complete units during coupling. */

#ifndef TRAIN_COUPLING_H
#define TRAIN_COUPLING_H

#include <cstdint>
#include <vector>

struct Train;

enum class CoupleContactEnd : uint8_t {
	Front,
	Back,
};

struct CoupleUnitSelection {
	bool possible;
	uint16_t first_unit;
	uint16_t unit_count;
	uint16_t remaining_unit_count;
};

struct CoupleTrainUnitSelection {
	bool possible;
	Train *selected_first;
	Train *remaining_first;
	uint16_t selected_unit_count;
	uint16_t remaining_unit_count;
	bool reverse_selected;
};

inline CoupleContactEnd SelectCoupleContactEnd(uint32_t distance_to_front, uint32_t distance_to_back)
{
	return distance_to_front <= distance_to_back ? CoupleContactEnd::Front : CoupleContactEnd::Back;
}

inline CoupleUnitSelection PlanCoupleUnitSelection(uint16_t available_units, uint8_t requested_units, CoupleContactEnd contact_end)
{
	const uint16_t take = requested_units == 0 ? available_units : requested_units;
	if (take > available_units) return {false, 0, 0, available_units};

	return {
		true,
		contact_end == CoupleContactEnd::Front ? uint16_t{0} : static_cast<uint16_t>(available_units - take),
		take,
		static_cast<uint16_t>(available_units - take),
	};
}

/** The source dispatch history is consumed only when coupling absorbs it completely. */
inline bool ShouldClearCoupleSourceDispatchRecords(bool complete_take)
{
	return complete_take;
}

CoupleTrainUnitSelection FindCoupleTrainUnitSelection(Train *waiting_first, uint8_t requested_units, CoupleContactEnd contact_end);
std::vector<Train *> FindReversibleCoupleParts(Train *first, Train *end);
void ReverseCouplePhysicalPartStates(std::vector<Train *> &physical_parts);
const Train *ResolveDepotSellAllTrain(const Train *listed);

#endif /* TRAIN_COUPLING_H */
