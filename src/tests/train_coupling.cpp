/*
 * This file is part of OpenTTD.
 * OpenTTD is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, version 2.
 * OpenTTD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with OpenTTD. If not, see <https://www.gnu.org/licenses/old-licenses/gpl-2.0>.
 */

/** @file train_coupling.cpp Tests for train coupling invariants. */

#include "../stdafx.h"

#include "../3rdparty/catch2/catch.hpp"

#include "../infrastructure_func.h"
#include "../sl/extended_ver_sl.h"
#include "../train.h"
#include "../train_coupling.h"
#include "../tracerestrict.h"

#include "../safeguards.h"

TEST_CASE("Cross-company coupling uses the moving train owner")
{
	const Owner moving_owner{1};
	const Owner waiting_owner{2};

	CHECK(GetTrainCouplingResultOwner(moving_owner, waiting_owner) == moving_owner);
}

TEST_CASE("Same-company coupling keeps its owner")
{
	const Owner owner{1};

	CHECK(GetTrainCouplingResultOwner(owner, owner) == owner);
}

TEST_CASE("Partial coupling takes the requested units from the contact end")
{
	CHECK(SelectCoupleContactEnd(0, 10) == CoupleContactEnd::Front);
	CHECK(SelectCoupleContactEnd(10, 0) == CoupleContactEnd::Back);
	CHECK(SelectCoupleContactEnd(0, 0) == CoupleContactEnd::Front);

	const CoupleUnitSelection front = PlanCoupleUnitSelection(3, 2, CoupleContactEnd::Front);
	CHECK(front.possible);
	CHECK(front.first_unit == 0);
	CHECK(front.unit_count == 2);
	CHECK(front.remaining_unit_count == 1);

	const CoupleUnitSelection back = PlanCoupleUnitSelection(3, 2, CoupleContactEnd::Back);
	CHECK(back.possible);
	CHECK(back.first_unit == 1);
	CHECK(back.unit_count == 2);
	CHECK(back.remaining_unit_count == 1);
}

TEST_CASE("Partial coupling waits when the target has fewer units than requested")
{
	const CoupleUnitSelection selection = PlanCoupleUnitSelection(1, 2, CoupleContactEnd::Front);
	CHECK_FALSE(selection.possible);
	CHECK(selection.unit_count == 0);
	CHECK(selection.remaining_unit_count == 1);
}

TEST_CASE("A zero coupling count takes the complete target")
{
	const CoupleUnitSelection selection = PlanCoupleUnitSelection(3, 0, CoupleContactEnd::Back);
	CHECK(selection.possible);
	CHECK(selection.first_unit == 0);
	CHECK(selection.unit_count == 3);
	CHECK(selection.remaining_unit_count == 0);

	_vehicle_pool.CleanPool();
	Train *first = Train::Create();
	Train *second = Train::Create();
	first->SetNext(second);
	CoupleTrainUnitSelection train_selection = FindCoupleTrainUnitSelection(first, 0, CoupleContactEnd::Back);
	CHECK(train_selection.reverse_selected);
	_vehicle_pool.CleanPool();
}

TEST_CASE("Partial coupling preserves the waiting dispatch history")
{
	CHECK_FALSE(ShouldClearCoupleSourceDispatchRecords(false));
	CHECK(ShouldClearCoupleSourceDispatchRecords(true));
}

TEST_CASE("Coupling selection never splits articulated or dual-headed units")
{
	_vehicle_pool.CleanPool();

	Train *articulated_base = Train::Create();
	Train *articulated_part = Train::Create();
	Train *wagon = Train::Create();
	SetBit(articulated_part->subtype, GVSF_ARTICULATED_PART);
	articulated_base->SetNext(articulated_part);
	articulated_part->SetNext(wagon);

	CoupleTrainUnitSelection articulated = FindCoupleTrainUnitSelection(articulated_base, 1, CoupleContactEnd::Front);
	REQUIRE(articulated.possible);
	CHECK(articulated.selected_first == articulated_base);
	CHECK(articulated.remaining_first == wagon);
	CHECK(articulated.selected_unit_count == 1);
	CHECK(articulated.remaining_unit_count == 1);

	_vehicle_pool.CleanPool();

	Train *dual_front = Train::Create();
	Train *dual_rear = Train::Create();
	Train *following_wagon = Train::Create();
	SetBit(dual_front->subtype, GVSF_ENGINE);
	SetBit(dual_front->subtype, GVSF_MULTIHEADED);
	SetBit(dual_rear->subtype, GVSF_MULTIHEADED);
	dual_front->other_multiheaded_part = dual_rear;
	dual_rear->other_multiheaded_part = dual_front;
	dual_front->SetNext(dual_rear);
	dual_rear->SetNext(following_wagon);

	CoupleTrainUnitSelection dual = FindCoupleTrainUnitSelection(dual_front, 1, CoupleContactEnd::Front);
	REQUIRE(dual.possible);
	CHECK(dual.selected_first == dual_front);
	CHECK(dual.remaining_first == following_wagon);
	CHECK(dual.selected_unit_count == 1);
	CHECK(dual.remaining_unit_count == 1);

	_vehicle_pool.CleanPool();
}

TEST_CASE("Back contact selects a complete suffix and leaves the physical prefix")
{
	_vehicle_pool.CleanPool();
	Train *first = Train::Create();
	Train *second = Train::Create();
	Train *third = Train::Create();
	first->SetNext(second);
	second->SetNext(third);

	CoupleTrainUnitSelection selection = FindCoupleTrainUnitSelection(first, 2, CoupleContactEnd::Back);
	REQUIRE(selection.possible);
	CHECK(selection.selected_first == second);
	CHECK(selection.remaining_first == first);
	CHECK(selection.selected_unit_count == 2);
	CHECK(selection.remaining_unit_count == 1);
	CHECK(selection.reverse_selected);

	_vehicle_pool.CleanPool();
}

TEST_CASE("Partial coupling refuses a cut across separated dual-headed halves")
{
	_vehicle_pool.CleanPool();
	Train *dual_front = Train::Create();
	Train *wrapped_wagon = Train::Create();
	Train *dual_rear = Train::Create();
	SetBit(dual_front->subtype, GVSF_ENGINE);
	SetBit(dual_front->subtype, GVSF_MULTIHEADED);
	SetBit(dual_rear->subtype, GVSF_MULTIHEADED);
	dual_front->other_multiheaded_part = dual_rear;
	dual_rear->other_multiheaded_part = dual_front;
	dual_front->SetNext(wrapped_wagon);
	wrapped_wagon->SetNext(dual_rear);

	CHECK_FALSE(FindCoupleTrainUnitSelection(dual_front, 1, CoupleContactEnd::Front).possible);
	CHECK_FALSE(FindCoupleTrainUnitSelection(dual_front, 1, CoupleContactEnd::Back).possible);
	CHECK(FindCoupleTrainUnitSelection(dual_front, 0, CoupleContactEnd::Front).possible);

	_vehicle_pool.CleanPool();
}

TEST_CASE("Coupling reversal excludes articulated parts without a physical direction")
{
	_vehicle_pool.CleanPool();
	Train *base = Train::Create();
	Train *visible_part = Train::Create();
	Train *hidden_part = Train::Create();
	base->direction = Direction::W;
	visible_part->direction = Direction::SW;
	hidden_part->direction = Direction::Invalid;
	base->track = TRACK_BIT_X;
	visible_part->track = TRACK_BIT_Y;
	SetBit(base->gv_flags, GVF_CHUNNEL_BIT);
	SetBit(visible_part->gv_flags, GVF_GOINGUP_BIT);
	visible_part->vehstatus.Set(VehState::Hidden);
	base->x_pos = 100;
	base->y_pos = 200;
	base->tile = TileIndex{10};
	base->z_pos = 3;
	visible_part->x_pos = 300;
	visible_part->y_pos = 400;
	visible_part->tile = TileIndex{20};
	visible_part->z_pos = 7;
	hidden_part->x_pos = 500;
	hidden_part->y_pos = 600;
	hidden_part->tile = TileIndex{30};
	hidden_part->z_pos = 11;
	hidden_part->vehstatus.Set(VehState::Hidden);
	hidden_part->gcache.cached_veh_length = 0;
	SetBit(visible_part->subtype, GVSF_ARTICULATED_PART);
	SetBit(hidden_part->subtype, GVSF_ARTICULATED_PART);
	base->SetNext(visible_part);
	visible_part->SetNext(hidden_part);

	std::vector<Train *> physical_parts = FindReversibleCoupleParts(base, nullptr);
	REQUIRE(physical_parts.size() == 2);
	CHECK(physical_parts[0] == base);
	CHECK(physical_parts[1] == visible_part);

	ReverseCouplePhysicalPartStates(physical_parts);
	CHECK(base->direction == Direction::NE);
	CHECK(visible_part->direction == Direction::E);
	CHECK(base->track == TRACK_BIT_Y);
	CHECK(visible_part->track == TRACK_BIT_X);
	CHECK(base->vehstatus.Test(VehState::Hidden));
	CHECK_FALSE(visible_part->vehstatus.Test(VehState::Hidden));
	CHECK(base->x_pos == 300);
	CHECK(base->y_pos == 400);
	CHECK(base->tile == TileIndex{20});
	CHECK(base->z_pos == 7);
	CHECK(visible_part->x_pos == 100);
	CHECK(visible_part->y_pos == 200);
	CHECK(visible_part->tile == TileIndex{10});
	CHECK(visible_part->z_pos == 3);
	CHECK(hidden_part->direction == Direction::Invalid);
	CHECK(hidden_part->x_pos == 500);
	CHECK(hidden_part->y_pos == 600);
	CHECK(hidden_part->tile == TileIndex{30});
	CHECK(hidden_part->z_pos == 11);
	CHECK(hidden_part->vehstatus.Test(VehState::Hidden));

	/* The rollback path reverses the selected chain a second time. */
	ReverseCouplePhysicalPartStates(physical_parts);
	CHECK(base->direction == Direction::W);
	CHECK(visible_part->direction == Direction::SW);
	CHECK(base->track == TRACK_BIT_X);
	CHECK(visible_part->track == TRACK_BIT_Y);
	CHECK_FALSE(base->vehstatus.Test(VehState::Hidden));
	CHECK(visible_part->vehstatus.Test(VehState::Hidden));
	CHECK(HasBit(base->gv_flags, GVF_CHUNNEL_BIT));
	CHECK(HasBit(visible_part->gv_flags, GVF_GOINGUP_BIT));
	CHECK(base->x_pos == 100);
	CHECK(visible_part->x_pos == 300);

	std::vector<Train *> no_parts;
	ReverseCouplePhysicalPartStates(no_parts);
	std::vector<Train *> one_part{base};
	ReverseCouplePhysicalPartStates(one_part);
	CHECK(base->direction == Direction::E);
	ReverseCouplePhysicalPartStates(one_part);
	CHECK(base->direction == Direction::W);

	_vehicle_pool.CleanPool();
}

TEST_CASE("Dual-headed coupling reversal skips non-physical articulated members")
{
	_vehicle_pool.CleanPool();
	Train *dual_front = Train::Create();
	Train *hidden_part = Train::Create();
	Train *dual_rear = Train::Create();
	dual_front->direction = Direction::W;
	hidden_part->direction = Direction::Invalid;
	dual_rear->direction = Direction::N;
	dual_front->x_pos = 100;
	hidden_part->x_pos = 200;
	dual_rear->x_pos = 300;
	hidden_part->vehstatus.Set(VehState::Hidden);
	hidden_part->gcache.cached_veh_length = 0;
	SetBit(dual_front->subtype, GVSF_ENGINE);
	SetBit(dual_front->subtype, GVSF_MULTIHEADED);
	SetBit(hidden_part->subtype, GVSF_ARTICULATED_PART);
	SetBit(dual_rear->subtype, GVSF_MULTIHEADED);
	dual_front->other_multiheaded_part = dual_rear;
	dual_rear->other_multiheaded_part = dual_front;
	dual_front->SetNext(hidden_part);
	hidden_part->SetNext(dual_rear);

	std::vector<Train *> physical_parts = FindReversibleCoupleParts(dual_front, nullptr);
	REQUIRE(physical_parts.size() == 2);
	ReverseCouplePhysicalPartStates(physical_parts);
	CHECK(dual_front->direction == Direction::S);
	CHECK(dual_front->x_pos == 300);
	CHECK(dual_rear->direction == Direction::E);
	CHECK(dual_rear->x_pos == 100);
	CHECK(hidden_part->direction == Direction::Invalid);
	CHECK(hidden_part->x_pos == 200);
	CHECK(hidden_part->vehstatus.Test(VehState::Hidden));

	_vehicle_pool.CleanPool();
}

TEST_CASE("Depot sell-all resolves a mid-chain identity to the physical head")
{
	_vehicle_pool.CleanPool();
	Train *wagon_head = Train::Create();
	Train *identity_engine = Train::Create();
	Train *tail_wagon = Train::Create();
	wagon_head->SetNext(identity_engine);
	identity_engine->SetNext(tail_wagon);
	SetBit(identity_engine->subtype, GVSF_ENGINE);
	identity_engine->SetFrontEngine();
	for (Train *part : {wagon_head, identity_engine, tail_wagon}) part->SetPrimary(identity_engine);

	CHECK(ResolveDepotSellAllTrain(identity_engine) == wagon_head);
	CHECK(ResolveDepotSellAllTrain(tail_wagon) == wagon_head);

	_vehicle_pool.CleanPool();
}

TEST_CASE("A wagon-headed train has one logical consist identity")
{
	_vehicle_pool.CleanPool();
	Train *wagon_head = Train::Create();
	Train *identity_engine = Train::Create();
	Train *tail_wagon = Train::Create();
	wagon_head->SetNext(identity_engine);
	identity_engine->SetNext(tail_wagon);
	SetBit(identity_engine->subtype, GVSF_ENGINE);
	wagon_head->SetFrontWagon();
	identity_engine->SetFrontEngine();

	for (Train *part = wagon_head; part != nullptr; part = part->Next()) part->SetPrimary(identity_engine);

	/* IsPrimaryVehicle retains its legacy physical-head meaning so existing
	 * IterateFrontOnly consumers continue to process this consist. */
	CHECK(wagon_head->IsPrimaryVehicle());
	REQUIRE(identity_engine->IsPrimaryVehicle());
	/* IsConsistIdentity is the unique logical predicate used for accounting. */
	CHECK_FALSE(wagon_head->IsConsistIdentity());
	CHECK(identity_engine->IsConsistIdentity());
	CHECK_FALSE(tail_wagon->IsConsistIdentity());
	CHECK(wagon_head->IsFrontWagon());
	CHECK(identity_engine->IsFrontEngine());

	/* Restore a conventional pool-destruction shape for this synthetic chain. */
	wagon_head->SetFrontWagon();
	_vehicle_pool.CleanPool();
}

TEST_CASE("Cross-company coupling keeps only slots usable by the survivor")
{
	const Owner moving_owner{1};
	const Owner waiting_owner{2};

	CHECK(IsTraceRestrictSlotUsableByOwner(waiting_owner, true, moving_owner));
	CHECK_FALSE(IsTraceRestrictSlotUsableByOwner(waiting_owner, false, moving_owner));
	CHECK(IsTraceRestrictSlotUsableByOwner(moving_owner, false, moving_owner));
}

TEST_CASE("Cross-company slot migration transfers usable slots and removes private ones")
{
	_tracerestrictslot_pool.CleanPool();
	TraceRestrictSlot::RebuildVehicleIndex();

	const Owner moving_owner{1};
	const Owner waiting_owner{2};
	const VehicleID waiting{10};
	const VehicleID survivor{20};

	TraceRestrictSlot *private_slot = TraceRestrictSlot::Create(waiting_owner);
	private_slot->occupants.push_back(waiting);
	TraceRestrictSlot *public_slot = TraceRestrictSlot::Create(waiting_owner);
	public_slot->flags.Set(TraceRestrictSlot::Flag::Public);
	public_slot->occupants.push_back(waiting);
	TraceRestrictSlot *owned_slot = TraceRestrictSlot::Create(moving_owner);
	owned_slot->occupants.push_back(waiting);
	TraceRestrictSlot *duplicate_slot = TraceRestrictSlot::Create(waiting_owner);
	duplicate_slot->flags.Set(TraceRestrictSlot::Flag::Public);
	duplicate_slot->occupants.push_back(waiting);
	duplicate_slot->occupants.push_back(survivor);
	TraceRestrictSlot::RebuildVehicleIndex();

	CHECK(TraceRestrictTransferVehicleOccupantInUsableSlots(waiting, survivor, moving_owner));
	CHECK_FALSE(private_slot->IsOccupant(waiting));
	CHECK_FALSE(private_slot->IsOccupant(survivor));
	CHECK(public_slot->IsOccupant(survivor));
	CHECK(owned_slot->IsOccupant(survivor));
	CHECK_FALSE(duplicate_slot->IsOccupant(waiting));
	CHECK(duplicate_slot->IsOccupant(survivor));
	CHECK(TraceRestrictSlot::ValidateVehicleIndex());

	_tracerestrictslot_pool.CleanPool();
	TraceRestrictSlot::RebuildVehicleIndex();
}

TEST_CASE("Legacy current-order decouple flags are selected only for pre-feature px saves")
{
	std::array<uint16_t, XSLFI_SIZE> features{};
	CHECK(ShouldLoadLegacyCurrentOrderDecoupleFlags(true, features));
	CHECK_FALSE(ShouldLoadLegacyCurrentOrderDecoupleFlags(false, features));

	features[XSLFI_ORDER_DECOUPLE] = 1;
	CHECK_FALSE(ShouldLoadLegacyCurrentOrderDecoupleFlags(true, features));
	features[XSLFI_ORDER_DECOUPLE] = 2;
	CHECK_FALSE(ShouldLoadLegacyCurrentOrderDecoupleFlags(true, features));
}
