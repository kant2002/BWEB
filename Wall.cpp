#include "Wall.h"
#include "AStar.h"
#include <tuple>
#include <chrono>

namespace BWEB
{
	Wall::Wall(const BWEM::Area * a, const BWEM::ChokePoint * c)
	{
		area = a;
		choke = c;
		wallDoor = TilePositions::Invalid;
	}

	void Map::createWall(vector<UnitType>& buildings, const BWEM::Area * area, const BWEM::ChokePoint * choke, UnitType tight, const vector<UnitType>& defenses)
	{
		if (!area || !choke || buildings.empty())
		{
			Broodwar << "this" << endl;
			return;
		}

		Wall newWall(area, choke);
		double distBest = DBL_MAX;
		const BWEM::Area* thirdArea = (choke->GetAreas().first != area) ? choke->GetAreas().first : choke->GetAreas().second;

		// Find our end position for A* pathfinding
		endTile = thirdArea != nullptr ? (TilePosition)thirdArea->Top() : TilePositions::Invalid;
		startTile = (TilePosition)mainArea->Top();

		// Sort buildings first
		sort(buildings.begin(), buildings.end());

		// If we don't care about tightness, just get best scored wall
		std::chrono::high_resolution_clock clock;
		chrono::steady_clock::time_point start;
		start = chrono::high_resolution_clock::now();

		// TODO: add dist to choke of overall wall to score
		if (iteratePieces(buildings, area, choke, tight))
		{
			for (auto& placement : bestWall)
				newWall.insertSegment(placement.first, placement.second);
		}

		currentWall = bestWall;

		if (!defenses.empty())
			addWallDefenses(defenses, newWall);
		walls.push_back(newWall);


		double dur = std::chrono::duration <double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();
		Broodwar << "Wall time: " << dur << endl;

		return;
	}

	bool Map::iteratePieces(vector<UnitType>& buildings, const BWEM::Area * area, const BWEM::ChokePoint * choke, UnitType tight)
	{
		// Sort functionality for Pylons by Hannes
		//std::sort(buildings.begin(), buildings.end(), [](UnitType l, UnitType r){ return (l == BWAPI::UnitTypes::Protoss_Pylon) < (r == BWAPI::UnitTypes::Protoss_Pylon); }); // Moves pylons to end
		//std::sort(buildings.begin(), find(buildings.begin(), buildings.end(), BWAPI::UnitTypes::Protoss_Pylon)); // Sorts everything before pylons

		Broodwar << buildings.begin()->c_str() << endl;

		sort(buildings.begin(), buildings.end());
		do {
			currentWall.clear();
			typeIterator = buildings.begin();
			placePiece((TilePosition)choke->Center(), buildings, area, choke, tight);
			if (!bestWall.empty())
				return true;

		} while (next_permutation(buildings.begin(), buildings.end()));

		return false;
	}

	bool Map::placePiece(TilePosition start, vector<UnitType>& buildings, const BWEM::Area * area, const BWEM::ChokePoint * choke, UnitType tight)
	{
		UnitType previousPiece = overlapsCurrentWall(start);
		TilePosition previous;
		TilePosition current((*typeIterator).tileSize());
		
		// If we have a previous piece, only iterate the pieces around it
		if (previousPiece != UnitTypes::None)
		{
			previous = (previousPiece.tileSize());

			int rPrevious = (previous.x * 16) - previousPiece.dimensionRight() - 1;
			int lCurrent = (current.x * 16) - (*typeIterator).dimensionLeft();

			int lPrevious = (previous.x * 16) - previousPiece.dimensionLeft();
			int rCurrent = (current.x * 16) - (*typeIterator).dimensionRight() - 1;

			// Left edge and right edge
			if ((rPrevious + lCurrent < 16) || (lPrevious + rCurrent < 16) || tight == UnitTypes::None)
			{
				int xLeft = start.x - current.x;
				int xRight = start.x + previous.x;
				for (int y = 1 + start.y - current.y; y < start.y + previous.y; y++)
				{
					TilePosition left(xLeft, y); TilePosition right(xRight, y);
					if (rPrevious + lCurrent < 16 || tight == UnitTypes::None)
						testPiece(left, buildings, area, choke, tight);
					if (lPrevious + rCurrent < 16 || tight == UnitTypes::None)
						testPiece(right, buildings, area, choke, tight);
				}
			}

			// Top and bottom edge
			int tPrevious = (previous.y * 16) - previousPiece.dimensionUp();
			int bCurrent = (current.y * 16) - (*typeIterator).dimensionDown() - 1;

			int bPrevious = (previous.y * 16) - previousPiece.dimensionDown() - 1;
			int tCurrent = (current.y * 16) - (*typeIterator).dimensionUp();

			if ((tPrevious + bCurrent < 16) || (bPrevious + tCurrent < 16) || tight == UnitTypes::None)
			{
				int yTop = start.y - current.y;
				int yBottom = start.y + previous.y;
				for (int x = 1 + start.x - current.x; x < start.x + previous.x; x++)
				{
					TilePosition top(x, yTop); TilePosition bot(x, yBottom);
					if (tPrevious + bCurrent < 16 || tight == UnitTypes::None)
						testPiece(top, buildings, area, choke, tight);
					if (bPrevious + tCurrent < 16 || tight == UnitTypes::None)
						testPiece(bot, buildings, area, choke, tight);
				}
			}
		}
		// Otherwise we need to start the choke center
		else
		{
			previous = TilePosition(choke->Center());
			for (int x = previous.x - 4; x < previous.x + 4; x++)
			{
				for (int y = previous.y - 4; y < previous.y + 4; y++)
				{
					TilePosition t(x, y);
					if (isWallTight((*typeIterator), t, tight))
						testPiece(t, buildings, area, choke, tight);
				}
			}
		}
		return true;
	}

	bool Map::testPiece(TilePosition t, vector<UnitType>& buildings, const BWEM::Area * area, const BWEM::ChokePoint * choke, UnitType tight)
	{
		// If this is not a valid type, not a valid tile, overlaps the current wall, overlaps anything, isn't within the area passed in, isn't placeable or isn't wall tight.
		if (!(*typeIterator).isValid()
			|| !t.isValid()
			|| overlapsCurrentWall(t, (*typeIterator).tileWidth(), (*typeIterator).tileHeight()) != UnitTypes::None
			|| overlapsAnything(t, (*typeIterator).tileWidth(), (*typeIterator).tileHeight(), true)
			|| tilesWithinArea(area, t, (*typeIterator).tileWidth(), (*typeIterator).tileHeight()) == 0
			|| !isPlaceable(*typeIterator, t)
			/*|| isWallTight(*typeIterator, t, tight)*/) return false;
		


		currentWall[t] = *typeIterator, typeIterator++;
		// If we haven't placed all the pieces yet, try placing another
		if (typeIterator != buildings.end())
		{
			placePiece(t, buildings, area, choke, tight);
			currentWall.erase(t), typeIterator--;
		}
		else
		{
			Broodwar << "yes" << endl;

			// Try to find a hole in the wall
			findCurrentHole(startTile, endTile, choke);
			double dist = 0.0;

			// If we're not fully walled, check score
			if (tight == UnitTypes::None)
			{
				for (auto& piece : currentWall)
					dist += piece.first.getDistance((TilePosition)choke->Center());
				if ((double)currentPath.size() / dist > bestWallScore)
				{
					bestWall = currentWall;
					bestWallScore = (double)currentPath.size() / dist;
				}
			}

			// If we are fully walled, check no hole
			else
			{
				if (!currentHole.isValid())
				{					
					for (auto& piece : currentWall)
						dist += piece.first.getDistance((TilePosition)choke->Center());
					if (dist < closest)
						bestWall = currentWall;
				}
			}

			// Erase current tile and reduce iterator
			currentWall.erase(t), typeIterator--;
		}
		return true;
	}

	void Map::findCurrentHole(TilePosition startTile, TilePosition endTile, const BWEM::ChokePoint * choke)
	{
		currentHole = TilePositions::None;
		currentPath = BWEB::AStar().findPath(startTile, endTile, true);

		// Quick check to see if the path contains our end point
		if (find(currentPath.begin(), currentPath.end(), endTile) == currentPath.end())
		{
			currentHole = TilePositions::None;
			return;
		}

		// Otherwise iterate all tiles and locate the hole
		for (auto& tile : currentPath)
		{
			double closestGeo = DBL_MAX;
			for (auto& geo : choke->Geometry())
			{
				if (overlapsCurrentWall(tile) == UnitTypes::None && TilePosition(geo) == tile && tile.getDistance(startTile) < closestGeo)
					currentHole = tile, closestGeo = tile.getDistance(startTile);
			}
		}
	}

	UnitType Map::overlapsCurrentWall(TilePosition here, int width, int height)
	{
		for (int x = here.x; x < here.x + width; x++)
		{
			for (int y = here.y; y < here.y + height; y++)
			{
				for (auto& placement : currentWall)
				{
					TilePosition tile = placement.first;
					if (x >= tile.x && x < tile.x + placement.second.tileWidth() && y >= tile.y && y < tile.y + placement.second.tileHeight())
						return placement.second;
				}
			}
		}
		return UnitTypes::None;
	}

	void Map::addToWall(UnitType building, Wall * wall, UnitType tight)
	{
		// TODO: Remake this
	}

	void Map::addWallDefenses(const vector<UnitType>& types, Wall& wall)
	{
		for (auto& building : types)
		{
			double distance = DBL_MAX;
			TilePosition tileBest = TilePositions::Invalid;
			TilePosition start(wall.getChokePoint()->Center());

			for (int x = start.x - 10; x <= start.x + 10; x++)
			{
				for (int y = start.y - 10; y <= start.y + 10; y++)
				{
					TilePosition t(x, y);

					if (!TilePosition(x, y).isValid()) continue;
					if (overlapsAnything(TilePosition(x, y), 2, 2, true) || overlapsWalls(t) || reservePath[x][y] > 0) continue;
					if (!isPlaceable(building, TilePosition(x, y)) || overlapsCurrentWall(TilePosition(x, y), 2, 2) != UnitTypes::None) continue;


					Position center = Position(TilePosition(x, y)) + Position(32, 32);
					Position hold = Position(start);
					double dist = center.getDistance(hold);

					if (BWEM::Map::Instance().GetArea(TilePosition(center)) != wall.getArea()) continue;
					if ((dist >= 128 || Broodwar->self()->getRace() == Races::Zerg) && dist < distance)
						tileBest = TilePosition(x, y), distance = dist;
				}
			}

			if (tileBest.isValid())
			{
				currentWall[tileBest] = building;
				wall.insertDefense(tileBest);
				start = tileBest;
			}
		}
		return;
	}

	//void Map::findPath(const BWEM::Area * start, const BWEM::Area * end)
	void Map::findPath()
	{
		// TODO: Add start->end, currently just does main->natural

		const BWEM::Area* thirdArea = (naturalChoke->GetAreas().first != naturalArea) ? naturalChoke->GetAreas().first : naturalChoke->GetAreas().second;
		TilePosition tile(thirdArea->Top());
		eraseBlock(tile);

		auto path = AStar().findPath((TilePosition)mainChoke->Center(), tile, false);
		for (auto& tile : path) {
			reservePath[tile.x][tile.y] = 1;
		}

		// Create a door for the path
		// TODO: Need a way better option than this
		if (naturalChoke)
		{
			double distBest = DBL_MAX;
			TilePosition tileBest(TilePositions::Invalid);
			for (auto &geo : naturalChoke->Geometry())
			{
				TilePosition tile = (TilePosition)geo;
				double dist = tile.getDistance((TilePosition)naturalChoke->Center());
				if (reservePath[tile.x][tile.y] == 1 && dist < distBest)
					tileBest = tile, distBest = dist;
			}
		}
	}



	//bool Map::isWallTight(UnitType building, TilePosition here, UnitType tight)
	//{
	//	L = R = T = B = false;
	//	walled = false;
	//	int walkHeight = building.tileHeight() * 4;
	//	int walkWidth = building.tileWidth() * 4;
	//	int halfTileHeight = building.tileHeight() * 16;
	//	int halfTileWidth = building.tileWidth() * 16;
	//	int pixelCheck = 16;// min(tight.width(), tight.height());

	//	if (halfTileHeight - building.dimensionDown() - 1 < pixelCheck)
	//		B = true;
	//	if (halfTileHeight - building.dimensionUp() < pixelCheck)
	//		T = true;
	//	if (halfTileWidth - building.dimensionLeft() < pixelCheck)
	//		L = true;
	//	if (halfTileWidth - building.dimensionRight() - 1 < pixelCheck)
	//		R = true;

	//	WalkPosition right = WalkPosition(here) + WalkPosition(walkWidth, 0);
	//	WalkPosition left = WalkPosition(here) - WalkPosition(1, 0);
	//	WalkPosition top = WalkPosition(here) - WalkPosition(0, 1);
	//	WalkPosition bottom = WalkPosition(here) + WalkPosition(0, walkHeight);

	//	for (int y = right.y; y < right.y + walkHeight; y++)
	//	{
	//		int x = right.x;
	//		int state = tightState(WalkPosition(x, y), building, R);
	//		if (state == 0)
	//			return false;
	//		if (state == 1)
	//			return true;
	//	}

	//	for (int y = left.y; y < left.y + walkHeight; y++)
	//	{
	//		int x = left.x;
	//		int state = tightState(WalkPosition(x, y), building, L);
	//		if (state == 0)
	//			return false;
	//		if (state == 1)
	//			return true;
	//	}

	//	for (int x = top.x; x < top.x + walkWidth; x++)
	//	{
	//		int y = top.y;
	//		int state = tightState(WalkPosition(x, y), building, T);
	//		if (state == 0)
	//			return false;
	//		if (state == 1)
	//			return true;
	//	}

	//	for (int x = bottom.x; x < bottom.x + walkWidth; x++)
	//	{
	//		int y = bottom.y;
	//		int state = tightState(WalkPosition(x, y), building, B);
	//		if (state == 0)
	//			return false;
	//		if (state == 1)
	//			return true;
	//	}

	//	if (walled)
	//		return true;
	//	return false;
	//}

	//int Map::tightState(WalkPosition w, UnitType building, bool side)
	//{
	//	TilePosition t(w);
	//	UnitType wallPiece = overlapsCurrentWall(t);

	//	if (BWEBUtil().overlapsStations(t) && currentWall.size() == 0)
	//		wallPiece = Broodwar->self()->getRace().getCenter();

	//	if (side && (currentWall.size() == 0 || Broodwar->self()->getRace() == Races::Protoss))
	//	{
	//		if (!w.isValid() || !Broodwar->isWalkable(w) || BWEBUtil().overlapsNeutrals(t)) return 1;
	//		if (reservePath[t.x][t.y] == 1) return 1;
	//	}
	//	if (wallPiece != UnitTypes::None)
	//	{
	//		int r = (building.tileWidth() * 16) - building.dimensionRight() - 1;
	//		int l = (wallPiece.tileWidth() * 16) - wallPiece.dimensionLeft();
	//		walled = true;

	//		if (l + r >= 16)
	//			return 0;
	//	}
	//	return 2;
	//}

	bool Map::isPoweringWall(TilePosition here)
	{
		for (auto& piece : currentWall)
		{
			TilePosition tile(piece.first);
			UnitType type(piece.second);
			if (type.tileWidth() == 4)
			{
				bool powersThis = false;
				if (tile.y - here.y == -5 || tile.y - here.y == 4)
				{
					if (tile.x - here.x >= -4 && tile.x - here.x <= 1) powersThis = true;
				}
				if (tile.y - here.y == -4 || tile.y - here.y == 3)
				{
					if (tile.x - here.x >= -7 && tile.x - here.x <= 4) powersThis = true;
				}
				if (tile.y - here.y == -3 || tile.y - here.y == 2)
				{
					if (tile.x - here.x >= -8 && tile.x - here.x <= 5) powersThis = true;
				}
				if (tile.y - here.y >= -2 && tile.y - here.y <= 1)
				{
					if (tile.x - here.x >= -8 && tile.x - here.x <= 6) powersThis = true;
				}
				if (!powersThis) return false;
			}
			else
			{
				bool powersThis = false;
				if (tile.y - here.y == 4)
				{
					if (tile.x - here.x >= -3 && tile.x - here.x <= 2) powersThis = true;
				}
				if (tile.y - here.y == -4 || tile.y - here.y == 3)
				{
					if (tile.x - here.x >= -6 && tile.x - here.x <= 5) powersThis = true;
				}
				if (tile.y - here.y >= -3 && tile.y - here.y <= 2)
				{
					if (tile.x - here.x >= -7 && tile.x - here.x <= 6) powersThis = true;
				}
				if (!powersThis) return false;
			}
		}
		return true;
	}

	void Wall::insertSegment(TilePosition here, UnitType building)
	{
		if (building.tileWidth() >= 4) large.insert(here);
		else if (building.tileWidth() >= 3) medium.insert(here);
		else small.insert(here);
	}

	const Wall * Map::getClosestWall(TilePosition here) const
	{
		double distBest = DBL_MAX;
		const Wall * bestWall = nullptr;
		for (auto& wall : walls)
		{
			double dist = here.getDistance((TilePosition)wall.getChokePoint()->Center());

			if (dist < distBest)
			{
				distBest = dist;
				bestWall = &wall;
			}
		}
		return bestWall;
	}

	const Wall* Map::getWall(const BWEM::Area * area, const BWEM::ChokePoint * choke)
	{
		if (!area && !choke) return nullptr;

		for (auto& wall : walls)
		{
			if ((!area || wall.getArea() == area) && (!choke || wall.getChokePoint() == choke))
				return &wall;
		}
		return nullptr;
	}




	bool Map::isWallTight(UnitType building, TilePosition here, UnitType type)
	{
		bool L, R, T, B;
		L = R = T = B = false;
		int height = building.tileHeight() * 4;
		int width = building.tileWidth() * 4;

		int htSize = building.tileHeight() * 16;
		int wtSize = building.tileWidth() * 16;

		if (type == UnitTypes::None)
			L = R = T = B = true;
		else
		{
			if (htSize - building.dimensionDown() - 1 < 16)
				B = true;
			if (htSize - building.dimensionUp() < 16)
				T = true;
			if (wtSize - building.dimensionLeft() < 16)
				L = true;
			if (wtSize - building.dimensionRight() - 1 < 16)
				R = true;
		}

		WalkPosition right = WalkPosition(here) + WalkPosition(width, 0);
		WalkPosition left = WalkPosition(here) - WalkPosition(1, 0);
		WalkPosition top = WalkPosition(here) - WalkPosition(0, 1);
		WalkPosition bottom = WalkPosition(here) + WalkPosition(0, height);

		bool walled = false;
		for (int y = right.y; y < right.y + height; y++)
		{
			int x = right.x;
			WalkPosition w(x, y);
			TilePosition t(w);
			UnitType wallPiece = overlapsCurrentWall(t);

			// TODO: Could reduce this to a tri-state function (tightState function below)
			// return false = 0, return true = 1, return nothing = 2
			if (R && (currentWall.size() == 0 || Broodwar->self()->getRace() == Races::Protoss))
			{
				if (!w.isValid() || !Broodwar->isWalkable(w) || overlapsNeutrals(t)) return true;
				if (reservePath[t.x][t.y] == 1) return true;
			}
			if (wallPiece != UnitTypes::None)
			{
				int r = (building.tileWidth() * 16) - building.dimensionRight() - 1;
				int l = (wallPiece.tileWidth() * 16) - wallPiece.dimensionLeft();
				walled = true;

				if (l + r >= 16)
					return false;
			}
		}

		for (int y = left.y; y < left.y + height; y++)
		{
			int x = left.x;
			WalkPosition w(x, y);
			TilePosition t(w);
			UnitType wallPiece = overlapsCurrentWall(t);

			if (L && (currentWall.size() == 0 || Broodwar->self()->getRace() == Races::Protoss))
			{
				if (!w.isValid() || !Broodwar->isWalkable(w) || overlapsNeutrals(t)) return true;
				if (reservePath[t.x][t.y] == 1) return true;
			}
			if (wallPiece != UnitTypes::None)
			{
				int l = (building.tileWidth() * 16) - building.dimensionLeft();
				int r = (wallPiece.tileWidth() * 16) - wallPiece.dimensionRight() - 1;
				walled = true;

				if (l + r >= 16)
					return false;
			}
		}

		for (int x = top.x; x < top.x + width; x++)
		{
			int y = top.y;
			WalkPosition w(x, y);
			TilePosition t(w);
			UnitType wallPiece = overlapsCurrentWall(t);

			if (T && (currentWall.size() == 0 || Broodwar->self()->getRace() == Races::Protoss))
			{
				if (!w.isValid() || !Broodwar->isWalkable(w) || overlapsNeutrals(t)) return true;
				if (reservePath[t.x][t.y] == 1) return true;
			}
			if (wallPiece != UnitTypes::None)
			{
				int u = (building.tileHeight() * 16) - building.dimensionUp();
				int d = (wallPiece.tileHeight() * 16) - wallPiece.dimensionDown() - 1;
				walled = true;

				if (u + d >= 16)
					return false;
			}
		}

		for (int x = bottom.x; x < bottom.x + width; x++)
		{
			int y = bottom.y;
			WalkPosition w(x, y);
			TilePosition t(w);
			UnitType wallPiece = overlapsCurrentWall(t);

			if (B && (currentWall.size() == 0 || Broodwar->self()->getRace() == Races::Protoss))
			{
				if (!w.isValid() || !Broodwar->isWalkable(w) || overlapsNeutrals(t)) return true;
				if (reservePath[t.x][t.y] == 1) return true;
			}
			if (wallPiece != UnitTypes::None)
			{
				int d = (building.tileHeight() * 16) - building.dimensionDown() - 1;
				int u = (wallPiece.tileHeight() * 16) - wallPiece.dimensionUp();
				walled = true;

				if (u + d >= 16)
					return false;
			}
		}

		if (walled)
			return true;

		return false;
	}
}