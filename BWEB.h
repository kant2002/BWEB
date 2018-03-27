#pragma once
#pragma warning(disable : 4351)
#include <set>

#include <BWAPI.h>
#include "..\BWEM\bwem.h"
#include "Station.h"
#include "Block.h"
#include "Wall.h"

namespace BWEB
{
	using namespace BWAPI;
	using namespace std;

	class Block;
	class Wall;
	class Station;
	class Map
	{
	private:

		vector<Station> stations;
		vector<Wall> walls;
		vector<Block> blocks;
		
		// Blocks
		void findStartBlock(), findHiddenTechBlock();
		bool canAddBlock(TilePosition, int, int, bool);
		void insertTinyBlock(TilePosition, bool, bool);
		void insertSmallBlock(TilePosition, bool, bool);
		void insertMediumBlock(TilePosition, bool, bool);
		void insertLargeBlock(TilePosition, bool, bool);		
		void insertStartBlock(TilePosition, bool, bool);
		void insertTechBlock(TilePosition, bool, bool);

		// Walls
		bool isWallTight(UnitType, TilePosition);
		bool isPlaceable(UnitType, TilePosition);
		bool isPoweringWall(TilePosition);

		int tightState(WalkPosition, UnitType, bool);
		void findCurrentHole(TilePosition, TilePosition, const BWEM::ChokePoint *);

		bool iteratePieces();
		bool checkPiece(TilePosition);
		bool testPiece(TilePosition);
		bool placePiece(TilePosition);
		bool identicalPiece(TilePosition, UnitType, TilePosition, UnitType);


		void addWallDefenses(const vector<UnitType>& type, Wall& wall);

		int reservePath[256][256] ={};
		int overlapGrid[256][256] ={};
		void addOverlap(TilePosition, int, int);

		bool walled, R, L, T, B;
		double closest = DBL_MAX;
		vector<TilePosition> currentPath;
		double bestWallScore = 0.0;
		TilePosition currentHole, startTile, endTile;
		vector<UnitType>::iterator typeIterator;		
		map<TilePosition, UnitType> bestWall;
		map<TilePosition, UnitType> currentWall;

		vector<UnitType> buildings;
		const BWEM::ChokePoint * choke;
		const BWEM::Area * area;
		UnitType tight;

		struct VisitGrid
		{
			int location[256][256] ={};
		};
		map<UnitType, VisitGrid> visited;
		bool parentSame, currentSame;
		double currentPathSize;
		
		
		// Map
		void findMain(), findMainChoke(), findNatural(), findNaturalChoke();
		Position mainPosition, naturalPosition;
		TilePosition mainTile, naturalTile;		
		const BWEM::Area * naturalArea;
		const BWEM::Area * mainArea;
		const BWEM::ChokePoint * naturalChoke;
		const BWEM::ChokePoint * mainChoke;
		set<TilePosition> usedTiles;

		// Stations
		void findStations();
		set<TilePosition>& stationDefenses(TilePosition, bool, bool);
		set<TilePosition> returnValues;

		// General
		static Map* BWEBInstance;

	public:
		void draw(), onStart(), onUnitDiscover(Unit), onUnitDestroy(Unit), onUnitMorph(Unit);
		static Map &Instance();

		/// This is just put here so AStar can use it for now
		UnitType overlapsCurrentWall(TilePosition tile, int width = 1, int height = 1);

		bool overlapsBlocks(TilePosition);
		bool overlapsStations(TilePosition);
		bool overlapsNeutrals(TilePosition);
		bool overlapsMining(TilePosition);
		bool overlapsWalls(TilePosition);
		bool overlapsAnything(TilePosition here, int width = 1, int height = 1, bool ignoreBlocks = false);
		bool isWalkable(TilePosition);
		int tilesWithinArea(BWEM::Area const *, TilePosition here, int width = 1, int height = 1);

		/// <summary> Returns the closest buildable TilePosition for any type of structure </summary>
		/// <param name="type"> The UnitType of the structure you want to build. </summary>
		/// <param name="tile"> The TilePosition you want to build closest to. </summary>
		TilePosition getBuildPosition(UnitType type, TilePosition tile = Broodwar->self()->getStartLocation());

		/// <summary> Returns the closest buildable TilePosition for a defensive structure </summary>
		/// <param name="type"> The UnitType of the structure you want to build. </summary>
		/// <param name="tile"> The TilePosition you want to build closest to. </summary>
		TilePosition getDefBuildPosition(UnitType type, TilePosition tile = Broodwar->self()->getStartLocation());

		template <class PositionType>
		/// <summary> Returns the estimated ground distance from one Position type to another Position type.</summary>
		/// <param name="first"> The first Position. </param>
		/// <param name="second"> The second Position. </param>
		double getGroundDistance(PositionType first, PositionType second);

		/// <summary> <para> Returns a pointer to a BWEB::Wall if it has been created in the given BWEM::Area and BWEM::ChokePoint. </para>
		/// <para> Note: If you only pass a BWEM::Area or a BWEM::ChokePoint (not both), it will imply and pick a BWEB::Wall that exists within that Area or blocks that BWEM::ChokePoint. </para></summary>
		/// <param name="area"> The BWEM::Area that the BWEB::Wall resides in </param>
		/// <param name="choke"> The BWEM::Chokepoint that the BWEB::Wall blocks </param>
		const Wall* getWall(BWEM::Area const* area = nullptr, BWEM::ChokePoint const* choke = nullptr);

		// TODO: Add this
		Station* getStation(BWEM::Area const* area);

		/// <summary> Returns the BWEM::Area of the natural expansion </summary>
		const BWEM::Area * getNaturalArea() { return naturalArea; }

		/// <summary> Returns the BWEM::Area of the main </summary>
		const BWEM::Area * getMainArea() { return mainArea; }

		/// <summary> Returns the BWEM::Chokepoint of the natural </summary>
		const BWEM::ChokePoint * getNaturalChoke() { return naturalChoke; }

		/// <summary> Returns the BWEM::Chokepoint of the main </summary>
		const BWEM::ChokePoint * getMainChoke() { return mainChoke; }

		/// <summary> Returns a vector containing every BWEB::Wall. </summary>
		vector<Wall> getWalls() const { return walls; }

		/// <summary> Returns a vector containing every BWEB::Block </summary>
		vector<Block> Blocks() const { return blocks; }

		/// <summary> Returns a vector containing every BWEB::Station </summary>
		vector<Station> Stations() const { return stations; }

		/// <summary> Returns the closest BWEB::Station to the given TilePosition. </summary>
		const Station* getClosestStation(TilePosition) const;

		/// <summary> Returns the closest BWEB::Wall to the given TilePosition. </summary>
		const Wall* getClosestWall(TilePosition) const;

		/// <summary> Returns the closest BWEB::Block to the given TilePosition. </summary>
		const Block* getClosestBlock(TilePosition) const;

		// Returns the TilePosition of the natural expansion
		TilePosition getNatural() { return naturalTile; }

		// Returns the set of used TilePositions
		set<TilePosition>& getUsedTiles() { return usedTiles; }

		/// <summary> <para> Given a vector of UnitTypes, an Area and a Chokepoint, finds an optimal wall placement, returns true if a valid BWEB::Wall was created. </para>
		/// <para> Note: Highly recommend that only Terran walls attempt to be walled tight, as most Protoss and Zerg wallins have gaps to allow your units through.</para>
		/// <para> BWEB makes tight walls very differently from non tight walls and will only create a tight wall if it is completely tight. </para></summary>
		/// <param name="buildings"> A Vector of UnitTypes that you want the BWEB::Wall to consist of. </param>
		/// <param name="area"> The BWEM::Area that you want the BWEB::Wall to be contained within. </param>
		/// <param name="choke"> The BWEM::Chokepoint that you want the BWEB::Wall to block. </param>
		/// <param name="tight"> (Optional) Decides whether this BWEB::Wall intends to be walled around a specific UnitType. </param>
		/// <param name="defenses"> A Vector of UnitTypes that you want the BWEB::Wall to have defenses consisting of. </param>
		void createWall(vector<UnitType>& buildings, const BWEM::Area * area, const BWEM::ChokePoint * choke, UnitType tight = UnitTypes::None, const vector<UnitType>& defenses ={});

		/// <summary> Adds a UnitType to a currently existing BWEB::Wall. </summary>
		/// <param name="type"> The UnitType you want to place at the BWEB::Wall. </param>
		/// <param name="area"> The BWEB::Wall you want to add to. </param>
		/// <param name="tight"> (Optional) Decides whether this addition to the BWEB::Wall intends to be walled around a specific UnitType. Defaults to none. </param>
		void addToWall(UnitType type, Wall* wall, UnitType tight = UnitTypes::None);
				
		//// <summary> This will create a path that walls cannot be built on, connecting your main choke to your natural choke. Call it only once. </summary>
		//void findPath(const BWEM::Area *, const BWEM::Area *);
		void findPath();

		/// <summary> Erases any blocks at the specified TilePosition. </summary>
		/// <param name="here"> The TilePosition that you want to delete any BWEB::Block that exists here. </summary>
		void eraseBlock(TilePosition here);

		/// <summary> Initializes the building of every BWEB::Block on the map, call it only once per game. </summary>
		void findBlocks();
	};

}