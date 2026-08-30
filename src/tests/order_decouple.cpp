/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file order_decouple.cpp Test decouple order option IDs and validation. */

#include "../stdafx.h"

#include "../3rdparty/catch2/catch.hpp"

#include "../order_type.h"

#include "../safeguards.h"

TEST_CASE("Decouple order dropdown uses explicit valid option IDs")
{
	REQUIRE(_valid_order_decouple_orders.size() == 4);
	CHECK(_valid_order_decouple_orders[0] == ODOF_KEEP_ORDERS);
	CHECK(_valid_order_decouple_orders[1] == ODOF_KEEP_ORDERS_NO_LOAD);
	CHECK(_valid_order_decouple_orders[2] == ODOF_WAIT_FOR_COUPLE);
	CHECK(_valid_order_decouple_orders[3] == ODOF_LOAD_AND_WAIT);
}

TEST_CASE("Decouple order option validation rejects retired and out-of-range IDs")
{
	for (OrderDecoupleOrdersFlags value : _valid_order_decouple_orders) {
		CHECK(IsValidOrderDecoupleOrdersFlags(value));
	}

	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(static_cast<OrderDecoupleOrdersFlags>(2)));
	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(ODOF_END));
	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(static_cast<OrderDecoupleOrdersFlags>(255)));
	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(256));
	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(257));
	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(259));
	CHECK_FALSE(IsValidOrderDecoupleOrdersFlags(260));
}
