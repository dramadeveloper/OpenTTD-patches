/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file order_compat.cpp Test compatibility conversion for legacy order type data. */

#include "../stdafx.h"

#include "../3rdparty/catch2/catch.hpp"

#include "../core/bitmath_func.hpp"
#include "../sl/saveload.h"
#include "../order_type.h"
#include "../vehicle_base.h"

#include "../safeguards.h"

NamedSaveLoadTable GetOrderDescription();

TEST_CASE("Legacy station order type layout is converted")
{
	const uint8_t legacy_type = OT_GOTO_STATION |
			(static_cast<uint8_t>(OrderStopLocation::FarEnd) << 4) |
			(ONSF_NO_STOP_AT_ANY_STATION << 6);

	const uint16_t converted = ConvertLegacyOrderType(legacy_type);

	CHECK(GB(converted, 0, 5) == OT_GOTO_STATION);
	CHECK(GB(converted, 5, 2) == static_cast<uint8_t>(OrderStopLocation::FarEnd));
	CHECK(GB(converted, 7, 2) == ONSF_NO_STOP_AT_ANY_STATION);
}

TEST_CASE("Legacy conditional comparator layout is converted")
{
	const uint8_t legacy_type = OT_CONDITIONAL |
			(static_cast<uint8_t>(OrderConditionComparator::MoreThanOrEqual) << 5);

	const uint16_t converted = ConvertLegacyOrderType(legacy_type);

	CHECK(GB(converted, 0, 5) == OT_CONDITIONAL);
	CHECK(GB(converted, 9, 3) == static_cast<uint8_t>(OrderConditionComparator::MoreThanOrEqual));
}

TEST_CASE("Legacy PX order type fields keep their 16-bit file width")
{
	const NamedSaveLoadTable orders = GetOrderDescription();
	REQUIRE_FALSE(orders.empty());
	CHECK((orders.front().save_load.conv & SLE_FILE_TYPE_MASK) == SLE_FILE_U16);

	const NamedSaveLoadTable vehicles = GetVehicleDescription(VehicleType::End);
	std::vector<const NamedSaveLoad *> current_order_types;
	for (const NamedSaveLoad &entry : vehicles) {
		if (std::string_view{entry.name} == "current_order.type") current_order_types.push_back(&entry);
	}
	REQUIRE(current_order_types.size() >= 2);
	CHECK((current_order_types[1]->save_load.conv & SLE_FILE_TYPE_MASK) == SLE_FILE_U16);
}
