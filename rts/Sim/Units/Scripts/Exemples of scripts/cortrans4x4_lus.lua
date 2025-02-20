base, link, link11, link12, link21, link22, thrust1, thrust2, thrust3, thrust4, thrust5, thrust6 = piece('base', 'link', 'link11', 'link12', 'link21', 'link22', 'thrust1', 'thrust2', 'thrust3', 'thrust4', 'thrust5', 'thrust6')
local SIG_AIM = {}

-- state variables
isMoving = "isMoving"
terrainType = "terrainType"

isLoading = false -- keep track of wether or not an animation is happening; needs work on that since right now it doesn't correspond to what's really happening
isUnloading = false

currentValue = 0 -- start with 0 transported units
maxValue = 4 -- max load values -- link to customparams in the future ?

local values = {}

for k,v in pairs (UnitDefs) do --store data on unitdefs values; for now based on xsize, but subject to a customparams change?
	if v.xsize <= 4 then
		values[k] = 1
	else
		values[k] = 4
	end
end

spots = { -- set list of transportation spots => 4 value 1 spots OR 1 value 4 spot
[1] = {pieceNum = link, value = 4, ["transporteeID"] = false},
[2] = {pieceNum = link11, value = 1, ["transporteeID"] = false},
[3] = {pieceNum = link12, value = 1, ["transporteeID"] = false},
[4] = {pieceNum = link21, value = 1, ["transporteeID"] = false},
[5] = {pieceNum = link22, value = 1, ["transporteeID"] = false},
}


dontmove = {  -- movectrl params table (don't move params)
maxSpeed = 0,
turnRate = 0,
accRate = 0,
altitudeRate = 0,
currentPitch = 0,
currentBank = 0,
}

move = Spring.GetUnitMoveTypeData (unitID ) -- movectrl params table (default params)

	
passengers = {} -- keep track of passengers IDs and their spot in transport (because list of transported units from engine will be inconsistent due to load/unload delays

function script.Create()
	Spring.SetUnitRadiusAndHeight(unitID, 1,1) -- because model's radius is too big so it cannot descend low enough to load small units, will have to be fixed in model but I have no model editor
end


function SetData(val)
	Spring.MoveCtrl.SetGunshipMoveTypeData( unitID, "maxSpeed", val.maxSpeed )
	Spring.MoveCtrl.SetGunshipMoveTypeData( unitID, "turnRate", val.turnRate )
	Spring.MoveCtrl.SetGunshipMoveTypeData( unitID, "accelRate", val.accRate )
	Spring.MoveCtrl.SetGunshipMoveTypeData( unitID, "altitudeRate", val.altitudeRate )
	Spring.MoveCtrl.SetGunshipMoveTypeData( unitID, "currentPitch", val.currentPitch )
	Spring.MoveCtrl.SetGunshipMoveTypeData( unitID, "currentBank", val.currentBank )
end

function script.StartMoving() -- unused here, i copied this from older armdfly script...
   isMoving = true
end

function script.StopMoving() -- unused here, i copied this from older armdfly script...
   isMoving = false
end   

function GetUnitValue(passengerID) -- Extract and return value of a unitID based on its unitDefID (maybe CanTransportLoadUnit and PerformLoad/Unload could send unitDefID as arg aswell, in order to not rely on Spring.GetUnitDefID() )
	return values[Spring.GetUnitDefID(passengerID)]
end

function GetSpot(passengerID, val) -- Assign (and return) pieceNum of the spot aswell as register transportee among transport
	for i = 1,5 do
	local tab = spots[i]
		if not (tab.transporteeID) then
			Spring.Echo(val, tab.value, passengerID)
			if val == tab.value then
				spots[i].transporteeID = passengerID
				passengers[passengerID] = i
				return spots[i].pieceNum
			end
		end
	end
	return false
end

function script.CanTransportUnloadNow() -- This will delay Unloading in case transport is busy
	return (isLoading == false and 0)
end

function script.CanTransportLoadNow() -- This will delay Loading in case transport is busy
	return (isUnloading == false and 0)
end

function script.CanTransportLoadUnit(passengerID)  -- This will unauthorize load a certain unit if should not be possible
	local val = GetUnitValue(passengerID)
	Spring.Echo(val, currentValue)
	if currentValue + val > maxValue then
		return 1
	end
	return 0
end

function script.IsTransportFull()  -- This will unauthorize load any unit if should not be possible
		-- Spring.Echo(currentValue)
	if currentValue == maxValue then
			-- Spring.Echo(currentValue, "is Full")

		return 0
	end
	return 1
end

function script.PassengerDied(passengerID) -- If a unit among transport dies; remove from passengers; /!\ for delayed attach and detach, a unit could be considered a passenger by script and not yet by engine, same for unload process. We might want to consider using Spring.IsValidUnitID() to cancel loading/unloading sequence midway when the unit died before attach or after detach.
	spots[passengers[passengerID]].transporteeID = false
	passengers[passengerID] = nil
	currentValue = currentValue - GetUnitValue(passengerID)
end

function script.PerformLoad ( passengerID ) -- New function does the actual loading (attach and all)
	local spot
	local val = GetUnitValue(passengerID)
	Spring.Echo(currentValue)
				spot = GetSpot(passengerID, GetUnitValue(passengerID))
				if spot then
					StartThread(StartTransport, passengerID,spot)
					currentValue = currentValue + val
				end
end


-- next are functions to turn transport piece -> passenger vector in world space to transport piece -> passenger vector in transporter's unit space
local function rotationMatrixX(rx)
    local cosx = math.cos(rx)
    local sinx = math.sin(rx)
    return {
        {1, 0, 0},
        {0, cosx, -sinx},
        {0, sinx, cosx}
    }
end


local function rotationMatrixY(ry)
    local cosy = math.cos(ry)
    local siny = math.sin(ry)
    return {
        {cosy, 0, siny},
        {0, 1, 0},
        {-siny, 0, cosy}
    }
end

local function rotationMatrixZ(rz)
    local cosz = math.cos(rz)
    local sinz = math.sin(rz)
    return {
        {cosz, -sinz, 0},
        {sinz, cosz, 0},
        {0, 0, 1}
    }
end

local function multiplyMatrices(a, b)
    local result = {}
    for i = 1, 3 do
        result[i] = {}
        for j = 1, 3 do
            result[i][j] = 0
            for k = 1, 3 do
                result[i][j] = result[i][j] + a[i][k] * b[k][j]
            end
        end
    end
    return result
end

local function applyRotation(matrix, vx, vy, vz)
    local x = matrix[1][1] * vx + matrix[1][2] * vy + matrix[1][3] * vz
    local y = matrix[2][1] * vx + matrix[2][2] * vy + matrix[2][3] * vz
    local z = matrix[3][1] * vx + matrix[3][2] * vy + matrix[3][3] * vz
    return x, y, z
end

function StartTransport(passengerID,spot)
	SetData(dontmove) -- first prevent the transport from trying to move, otherwise it will give very weird effects
	isLoading = true
	local x,y,z = Spring.GetUnitPiecePosDir(unitID, spot) -- transport piece position in world space
	local rx,ry,rz = Spring.GetUnitRotation(unitID) -- transporter rotation in worldspace
	local px,py,pz = Spring.GetUnitPosition(passengerID) -- passenger position in world space
	local rux,ruy,ruz = Spring.GetUnitRotation(passengerID) -- passenger rotation in world space
	local h = Spring.GetUnitHeight(passengerID) -- passenger height
	local dx,dy,dz = px-x, py-y, pz-z -- transport piece -> passenger vector in world space
	local rotX = rotationMatrixX(rx)
	local rotY = rotationMatrixY(ry)
	local rotZ = rotationMatrixZ(rz)
	local combinedRotation = multiplyMatrices(rotZ, multiplyMatrices(rotY, rotX))
	local dx, dy, dz = applyRotation(combinedRotation, dx, dy, dz) -- transport piece -> passenger vector in transporter's unit space
	local drx,dry,drz = rx - rux, ry-ruy, rz-ruz -- transport piece -> passenger rotation in unit space
	
	-- instantaneous move and turn to fit passenger position
	Move(spot, 1, dx)
	Move(spot, 2, dy)
	Move(spot, 3, dz)
	Turn(spot, 1, drx)
	Turn(spot, 2, dry)
	Turn(spot, 3, drz)
	
	Spring.UnitAttach(unitID, passengerID, spot) -- attach piece
	
	-- 3s load animation => go back to spot's position, and make sure height fits
	Move(spot, 1, 0, dx / 3)
	Move(spot, 2, -h, (dy+h) / 3)
	Move(spot, 3, 0, dz / 3)
	Turn(spot, 1, 0, drx/3)
	Turn(spot, 2, 0, dry/3)
	Turn(spot, 3, 0, drz/3)
	
	-- wait for the defined time (3 seconds since the animation takes 3 seconds)
	Sleep(3000)
	
	SetData(move) -- revert mvctrl params so the unit can move again
	isLoading = false
end

function DetachOne(passengerID)
	isUnloading = true
	local spot = spots[passengers[passengerID]].pieceNum
	SetData(dontmove) -- first prevent the transport from trying to move, otherwise it will give very weird effects
	local x,y,z = Spring.GetUnitPiecePosDir(unitID, spot) -- transport piece position in world space
	local h = Spring.GetGroundHeight(x,z)
	local hp = Spring.GetUnitHeight(passengerID)
	local dy = y- h + hp

	Move(spot, 2, -dy, (dy-hp) / 3)
	Sleep(3000)
	Spring.UnitDetach(passengerID)
	Spring.SetUnitPosition(passengerID, x,h,z)
	Move(spot, 2, 0)
	Move(spot, 1, 0)
	Move(spot, 3, 0)
	SetData(move) -- revert mvctrl params so the unit can move again
	spots[passengers[passengerID]].transporteeID = false
	passengers[passengerID] = nil
	currentValue = currentValue - GetUnitValue(passengerID)
	isUnloading = false
end

function script.PerformUnload(passengerID) -- New function that does the unloading
	Spring.Echo(passengerID)
				StartThread(DetachOne, passengerID)
end


local function RestoreAfterDelay() -- unused
end		


function script.Killed() -- unused
		return 1
end