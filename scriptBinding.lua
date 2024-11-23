--[[

ScriptBindingToLua v1.1.0 - Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code

Author: w33zl / WZL Modding
GitHub: github.com/w33zl/FS25-Script-Binding-Converter

Game version: 1.2.1.0

]]

--- Add a sample element to an AudioSource
---@param audioSource entityId "audioSource"
---@param filename string "filename"
---@param probability number "probability"
function addAudioSourceSampleElement2(audioSource, filename, probability) end

--- Add differential
---@param objectId entityId "objectId"
---@param diff0Index integer "diff0Index"
---@param diffIndex0IsWheel boolean "diffIndex0IsWheel"
---@param diff1Index integer "diff1Index"
---@param diffIndex1IsWheel boolean "diffIndex1IsWheel"
---@param ratio number "ratio"
---@param bias number "bias"
function addDifferential(objectId, diff0Index, diffIndex0IsWheel, diff1Index, diffIndex1IsWheel, ratio, bias) end

--- Load a new foliage type from an XML file, creating a new multilayer if a new density map is used
---@param terrainNode entityId
---@param foliageDataPlaneId entityId "id for density map to use for the foliage layer, or a dataplane that shares this density map"
---@param name string "name of new layer"
---@param xmlFilename string "XML filename containing the layer definition"
---@return densityMapTypeId entityId "the type ID the density map will use for this layer, or 0 if the call failed"
function addFoliageTypeFromXML(terrainNode, foliageDataPlaneId, name, xmlFilename) end

--- Add force to object (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@param forceX number "forceX"
---@param forceY number "forceY"
---@param forceZ number "forceZ"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
---@param isPositionLocal boolean "isPositionLocal"
function addForce(transformId, forceX, forceY, forceZ, positionX, positionY, positionZ, isPositionLocal) end

--- Add impulse to object (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@param impulseX number "impulseX"
---@param impulseY number "impulseY"
---@param impulseZ number "impulseZ"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
---@param isPositionLocal boolean "isPositionLocal"
function addImpulse(transformId, impulseX, impulseY, impulseZ, positionX, positionY, positionZ, isPositionLocal) end

--- (Recursively) adds given entity or its shape children to the ignore list for merged shadow rendering
---@param lightId entityId "id of the light source node"
---@param id shapeId "of shape or parent node of a set of shapes (such as a vehicle)."
function addMergedShadowIgnoreShapes(lightId, id) end

--- Add particle system simulation time
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param dt number "dt"
function addParticleSystemSimulationTime(particleSystemId, dt) end

--- Add a spline attribute
---@param shapeId integer
---@param attributeName string
---@return attributeIndex integer
function addSplineAttribute(shapeId, attributeName) end

--- Add terrain fill layer to given terrain
---@param terrainId entityId "terrainId"
---@param name string "fill type name"
---@param diffuseTexture string "diffuse texture filename"
---@param normalTexture string "normal texture filename"
---@param heightTexture string "height texture filename"
---@param displacementTexture string "displacement texture filename"
---@param layerUnitSize number "size of texture in worldspace (4.0 is typical)"
---@param displacementMaxHeight number "maximum height (+ or -) for displacement texture, in meters"
---@param blendContrast number "contrast value for blending (0.5 is typical)"
---@param noiseScale number "noise scale for blending (0.5 is typical)"
---@param noiseSharp number "noise sharpness for blending (0.25 is typical)"
---@param porosityAtZeroRoughness number "porosity when roughness is 0"
---@param porosityAtFullRoughness number "porosity when roughness is 1"
---@param firmness number "firmness of the ground 0-1 (used by tyre tracks to determine max depression)"
---@param viscosity number "viscosity of the ground 0-1 (used by tyre tracks to determine rate of sinking)"
---@param firmnessWet number "firmness of the ground when the terrain is wet 0-1"
function addTerrainFillLayer(terrainId, name, diffuseTexture, normalTexture, heightTexture, displacementTexture, layerUnitSize, displacementMaxHeight, blendContrast, noiseScale, noiseSharp, porosityAtZeroRoughness, porosityAtFullRoughness, firmness, viscosity, firmnessWet) end

--- Add to physics
---@param transformId entityId "transformId"
function addToPhysics(transformId) end

--- Adds torque to a collision (only for dynamic physics objects)
---@param objectId entityId "objectId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function addTorque(objectId, x, y, z) end

--- Adds torque impulse to a collision (only for dynamic physics objects)
---@param objectId entityId "objectId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function addTorqueImpulse(objectId, x, y, z) end

--- Adds a tire track position
---@param tyreTrackSystem entityId "tyreTrackSystem"
---@param trackId integer "trackId"
---@param x number "x coordinate"
---@param y number "y coordinate"
---@param z number "z coordinate"
---@param ux number "up direction x"
---@param uy number "up direction y"
---@param uz number "up direction z"
---@param r number "red"
---@param g number "green"
---@param b number "blue"
---@param a number "alpha"
---@param bumpiness number
---@param dTheta number "wheel dTheta/dT (used to determine forward/backward motion of wheel)"
---@param terrainOnly boolean "omit from geometry (only render on terrain)"
---@param colorBlendWithTerrain number "defines how the color should be blended with the terrain (0:100% terrain color, 1:0% terrain color)"
function addTrackPoint(tyreTrackSystem, trackId, x, y, z, ux, uy, uz, r, g, b, a, bumpiness, dTheta, terrainOnly, colorBlendWithTerrain) end

--- Add vehicle link
---@param transformId entityId "transformId"
---@param transformId2 entityId "transformId2"
function addVehicleLink(transformId, transformId2) end

--- Add comment to XML element.
---@param xmlId entityId "xmlId"
---@param xmlPath string "Path to element"
---@param comment string "comment"
function addXMLComment(xmlId, xmlPath, comment) end

--- Aim camera (spring/damper)
---@param cameraId entityId "cameraId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param distance number "distance"
---@param dt number "dt"
---@param springStrength number "springStrength"
function aimCamera(cameraId, x, y, z, distance, dt, springStrength) end

--- Converts an ascii latin1 (ISO 88859-1) encoded string to an utf8 string
---@param asciiString string "asciiString"
---@return utf8string string "utf8string"
function asciiToUtf8(asciiString) end

--- Assign clip to animation track
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@param clipIndex integer "clipIndex"
function assignAnimTrackClip(characterSetId, trackId, clipIndex) end

--- base64 decode
---@param input string "input"
---@return output byteArray "output"
function base64Decode(input) end

--- base64 encode
---@param input byteArray "input"
---@return output string "output"
function base64Encode(input) end

--- bit AND-Operation
---@param value1 integer "value1"
---@param value2 integer "value2"
---@return result integer "result"
function bitAND(value1, value2) end

--- bit HighestSet-Operation
---@param input integer "input"
---@return highestBit integer "highestBit"
function bitHighestSet(input) end

--- bit OR-Operation
---@param value1 integer "value1"
---@param value2 integer "value2"
---@return result integer "result"
function bitOR(value1, value2) end

--- bit ShiftRight-Operation
---@param value1 integer "value1"
---@param value2 integer "value2"
---@return result integer "result"
function bitShiftRight(value1, value2) end

--- Build the navigation mesh based on the specified world data
---@param navMeshId entityId "navMeshId"
---@param worldNode entityId "worldNode"
---@param cellSize number "cellSize"
---@param cellHeight number "cellHeight"
---@param agentHeight number "agentHeight"
---@param agentRadius number "agentRadius"
---@param agentMaxClimb number "agentMaxClimb"
---@param agentMaxSlope number "agentMaxSlope"
---@param minRegionSize number "minRegionSize"
---@param mergeRegionSize number "mergeRegionSize"
---@param maxEdgeLength number "maxEdgeLength"
---@param maxSimplificationError number "maxSimplificationError"
---@param navMeshBuildMask integer "navMeshBuildMask"
---@param terrainDetail number "terrainDetail"
---@param terrainCullInfoLayer string "terrainCullInfoLayer"
---@param terrainCullInfoLayerChannels integer "terrainCullInfoLayerChannels"
---@return success boolean "success"
function buildNavMesh(navMeshId, worldNode, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, minRegionSize, mergeRegionSize, maxEdgeLength, maxSimplificationError, navMeshBuildMask, terrainDetail, terrainCullInfoLayer, terrainCullInfoLayerChannels) end

--- Build the navigation mesh based on the provided contour
---@param navMeshId entityId
---@param contourWorldPositions floatArray
---@param terrainNodeId entityId
---@param collisionMask integer
---@param cellSize number "cellSize"
---@param cellHeight number "cellHeight"
---@param agentHeight number "agentHeight"
---@param agentRadius number "agentRadius"
---@param agentMaxClimb number "agentMaxClimb"
---@param agentMaxSlope number "agentMaxSlope"
---@param minRegionSize number "minRegionSize"
---@param mergeRegionSize number "mergeRegionSize"
---@param maxEdgeLength number "maxEdgeLength"
---@param maxSimplificationError number "maxSimplificationError"
---@return success boolean
function buildNavMeshFromContour(navMeshId, contourWorldPositions, terrainNodeId, collisionMask, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, minRegionSize, mergeRegionSize, maxEdgeLength, maxSimplificationError) end

--- Build the navigation mesh based on the specified shapes and provided contour
---@param navMeshId entityId
---@param shapesNodeId entityId
---@param shapesMask integer
---@param contourWorldPositions floatArray
---@param terrainNodeId entityId
---@param collisionMask integer
---@param cellSize number
---@param cellHeight number
---@param agentHeight number
---@param agentRadius number
---@param agentMaxClimb number
---@param agentMaxSlope number
---@param minRegionSize number
---@param mergeRegionSize number
---@param maxEdgeLength number
---@param maxSimplificationError number
---@return success boolean
function buildNavMeshFromShapesAndContour(navMeshId, shapesNodeId, shapesMask, contourWorldPositions, terrainNodeId, collisionMask, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, minRegionSize, mergeRegionSize, maxEdgeLength, maxSimplificationError) end

--- Calculate distance between two nodes
---@param transformOne entityId "transformId of first node"
---@param transformTwo entityId "transformId of second node"
---@return distance float "distance between the two nodes"
function calcDistanceFrom(transformOne, transformTwo) end

--- Calculate squared distance between two transforms
---@param transformOne entityId "transformId of first node"
---@param transformTwo entityId "transformId of second node"
---@return squaredDistance float "squared distance between the two nodes"
function calcDistanceSquaredFrom(transformOne, transformTwo) end

--- Cancel streaming I3D file
---@param requestId integer "request id from streamI3DFile"
function cancelStreamI3DFile(requestId) end

--- Clear animation track clip assignment
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
function clearAnimTrackClip(characterSetId, trackId) end

--- Clears the shared i3d cache, deleting all data and calling finish callbacks on all pending loads with failedReason = Cancelled.

function clearEntireSharedI3DFileCache() end

--- Clears (removes) all ignore shapes for merged shadow of the given light
---@param lightId entityId "id of the light source node"
---@param parameter2 any
function clearMergedShadowIgnoreShapes(lightId, parameter2) end

--- Clears all overlays in the given area.
--- Only allowed to be called within "draw"
---@param x number "x"
---@param y number "y"
---@param width number "width"
---@param height number "height"
---@param rotation number "rotation"
---@param rotCenterX number "center of rotation x"
---@param rotCenterY number "center of rotation y"
function clearOverlayArea(x, y, width, height, rotation, rotCenterX, rotCenterY) end

--- Clear terrain fill layers in given terrain
---@param terrainId entityId "terrainId"
function clearTerrainFillLayers(terrainId) end

--- Clone scenegraph object
---@param objectId entityId "objectId"
---@param groupUnderParent boolean "groupUnderParent"
---@param callOnCreate boolean|nil "callOnCreate [optional]"
---@param addPhysics boolean|nil "addPhysics [optional]"
---@return cloneId entityId "cloneId"
function clone(objectId, groupUnderParent, callOnCreate, addPhysics) end

--- Clone anim character set
---@param objectId entityId "objectId"
---@param targetId entityId "targetId"
---@return success boolean "success"
function cloneAnimCharacterSet(objectId, targetId) end

--- Calculate wheel shape tire forces
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@param longSlipRatio number "longSlipRatio"
---@param latSlipAngle number "latSlipAngle"
---@param tireLoad number "tireLoad"
---@return longForce float "longForce"
function computeWheelShapeTireForces(transformId, wheelShapeIndex, longSlipRatio, latSlipAngle, tireLoad) end

--- Debug draw
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function conditionalAnimationDebugDraw(conditionalAnimationEntityId, x, y, z) end

--- Register parameter
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param parameterId integer
---@param parameterType integer
---@param parameterLabel string
function conditionalAnimationRegisterParameter(conditionalAnimationEntityId, parameterId, parameterType, parameterLabel) end

--- Zeroise track times
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
function conditionalAnimationZeroiseTrackTimes(conditionalAnimationEntityId) end

--- Create audio source object for 3D sounds
---@param audioSourceName string "audioSourceName"
---@param sampleFilename string "sampleFilename"
---@param radius number "radius"
---@param innerRadius number "innerRadius"
---@param volume number "volume"
---@param loops integer "loops"
---@return audioSource entityId
function createAudioSource(audioSourceName, sampleFilename, radius, innerRadius, volume, loops) end

--- Create camera
---@param cameraName string "cameraName"
---@param fovy number "fovy"
---@param nearClip number "nearClip"
---@param farClip number "farClip"
---@return cameraId entityId "cameraId"
function createCamera(cameraName, fovy, nearClip, farClip) end

--- Create character controller (y axis capsule based)
---@param transformId entityId "transformId"
---@param radius number "radius"
---@param height number "height"
---@param stepOffset number "stepOffset"
---@param slopeLimit number "slopeLimit (in degrees)"
---@param skinWidth number "skinWidth"
---@param collisionGroup integer "collisionGroup"
---@param collisionMask integer "collisionMask"
---@param mass number "mass"
---@return characterIndex integer "characterIndex"
function createCCT(transformId, radius, height, stepOffset, slopeLimit, skinWidth, collisionGroup, collisionMask, mass) end

--- Create conditional animation
---@return conditionalAnimationEntityId entityId
function createConditionalAnimation() end

--- Creates a fill plane shape based on shapeId
---@param shapeId entityId "shapeId"
---@param shapeName string "shapeName"
---@param volume number "volume"
---@param deltaMax number "deltaMax"
---@param maxSurfaceAngle number "maxSurfaceAngle"
---@param maxPhysicalSurfaceAngle number "maxPhysicalSurfaceAngle"
---@param maxSurfaceDistanceError number "maxSurfaceDistanceError"
---@param maxSubDivEdgeLength number "maxSubDivEdgeLength"
---@param syncMaxSubDivEdgeLength number "syncMaxSubDivEdgeLength"
---@param createSidePlanes boolean "createSidePlanes"
---@param retessellateTop boolean "retessellateTop"
---@return fillPlaneShapeId entityId "fillPlaneShapeId"
function createFillPlaneShape(shapeId, shapeName, volume, deltaMax, maxSurfaceAngle, maxPhysicalSurfaceAngle, maxSurfaceDistanceError, maxSubDivEdgeLength, syncMaxSubDivEdgeLength, createSidePlanes, retessellateTop) end

--- 
---@param foliageBendingSystemId entityId
---@param minX number
---@param maxX number
---@param minZ number
---@param maxZ number
---@param yOffset number
---@param centerTransformid entityId
---@return rectangleId integer
function createFoliageBendingRectangle(foliageBendingSystemId, minX, maxX, minZ, maxZ, yOffset, centerTransformid) end

--- Create overlay object
---@param textureFilename string "textureFilename"
---@return imageOverlay entityId
function createImageOverlay(textureFilename) end

--- Create overlay object with existing texture
---@param textureId entityId "textureId"
---@return imageOverlay entityId
function createImageOverlayWithTexture(textureId) end

--- Create a new light source
---@param name string
---@param lightType integer "one of enum LightType"
---@param r number
---@param g number
---@param b number
---@param range number
---@return lightEntity entityId
function createLightSource(name, lightType, r, g, b, range) end

--- Create a loop synthesis sample object
---@param objectName string "objectName"
---@return sampleId entityId "sampleId"
function createLoopSynthesisSample(objectName) end

--- Create a navigation mesh node.
---@param name string "name"
---@return navMeshId entityId "navMeshId"
function createNavMesh(name) end

--- Create note node
---@param parentId entityId|nil "Parent ID [optional] otherwise linked to rootNode"
---@param text string|nil "Text [optional]"
---@param colorR float|nil "R component of note color [optional, 0-1]"
---@param colorG float|nil "G component of note color [optional, 0-1]"
---@param colorB float|nil "B component of note color [optional, 0-1]"
---@param fixedSize boolean|nil "Fixed size on screen [optional]"
---@return noteNodeId entityId
function createNoteNode(parentId, text, colorR, colorG, colorB, fixedSize) end

--- Create plane shape from 2D contour
---@param name string
---@param contour2DWorldPositions floatArray
---@param createRigidBody boolean|nil "[optional] defaults to false"
---@return shape entityId
function createPlaneShapeFrom2DContour(name, contour2DWorldPositions, createRigidBody) end

--- Create sample object
---@param objectName string "objectName"
---@return sampleId entityId "sampleId"
function createSample(objectName) end

--- Creates a shallow water simulation
---@param name string "name"
---@param gridWidth integer "Number of simulation grid cells in the x direction"
---@param gridHeight integer "Number of simulation grid cells in the y direction"
---@param physicalWidth number "Physical (world) width of the whole grid"
---@param physicalHeight number "Physical (world) height/length of the whole grid"
---@param useHeightDisplacement boolean|nil "[optional] Determines whether the simulation will make use of the height displacement of the water surface for rendering. Setting this to false assumes the output will only be used to modify water surface normals. Defaults to false"
---@return shallowWaterSimulationId entityId "ID of the created simulation"
function createShallowWaterSimulation(name, gridWidth, gridHeight, physicalWidth, physicalHeight, useHeightDisplacement) end

--- Creates a new spline entity from the given edit points.
---@param parentId entityId "The id of the entity which should be the parent of the newly created spline."
---@param editPoints floatArray "A list of edit point coordinates. Each edit point is defined by three consecutive coordinates (x, y, z). The number of supplied coordinates must be divisible by three. At least two edit points (six coordinates) must be supplied."
---@param makeLinearSpline boolean|nil "If true, the newly created spline is of linear type. Otherwise its of cubic type. [optional, default=false]"
---@param isClosed boolean|nil "If true, the newly created spline is closed. Otherwise its open. [optional, default=false]"
---@return objectId entityId "The id of the newly created spline entity."
function createSplineFromEditPoints(parentId, editPoints, makeLinearSpline, isClosed) end

--- Create streamed sample object
---@param objectName string "objectName"
---@param isBackgroundMusic boolean "isBackgroundMusic"
---@return streamedSampleId entityId "streamedSampleId"
function createStreamedSample(objectName, isBackgroundMusic) end

--- Creates a tire track system
---@param tyreTrackSystem entityId "tyreTrackSystem"
---@param width number "width"
---@param atlasIndex integer "atlasIndex"
---@return trackid integer "trackid"
function createTrack(tyreTrackSystem, width, atlasIndex) end

--- Create transform group
--- Use link() to place/move transform in the scenegraph
---@param transformName string "transformName"
---@return transformId entityId "transformId"
function createTransformGroup(transformName) end

--- Creates a tire track system
---@param rootNode entityId
---@param shape entityId "shape with tire track material"
---@param maxNumTracks integer
---@param maxNumSegments integer
---@param atlasSize integer
---@param terrainNode entityId
---@return tyreTrackSystemEntity entityId
function createTyreTrackSystem(rootNode, shape, maxNumTracks, maxNumSegments, atlasSize, terrainNode) end

--- Create wheel shape
---@param transformId entityId "transformId"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
---@param radius number "radius"
---@param suspensionTravel number "suspensionTravel"
---@param spring number "spring"
---@param damperCompressionLowSpeed number "damperCompressionLowSpeed"
---@param damperCompressionHighSpeed number "damperCompressionHighSpeed"
---@param damperCompressionLowSpeedThreshold number "damperCompressionLowSpeedThreshold"
---@param damperRelaxationLowSpeed number "damperRelaxationLowSpeed"
---@param damperRelaxationHighSpeed number "damperRelaxationHighSpeed"
---@param damperRelaxationLowSpeedThreshold number "damperRelaxationLowSpeedThreshold"
---@param mass number "mass"
---@param collisionGroup integer "collisionGroup"
---@param collisionMask integer "collisionMask"
---@param wheelShapeIndex integer "wheelShapeIndex (if 0, will create a new wheel)"
---@return wheelShapeIndex integer "wheelShapeIndex"
function createWheelShape(transformId, positionX, positionY, positionZ, radius, suspensionTravel, spring, damperCompressionLowSpeed, damperCompressionHighSpeed, damperCompressionLowSpeedThreshold, damperRelaxationLowSpeed, damperRelaxationHighSpeed, damperRelaxationLowSpeedThreshold, mass, collisionGroup, collisionMask, wheelShapeIndex) end

--- Create an empty XML file
---@param objectName string "objectName"
---@param filename string "filename"
---@param rootNodeName string "rootNodeName"
---@return xmlId integer "xmlId"
function createXMLFile(objectName, filename, rootNodeName) end

--- Relinquish track segments
---@param tyreTrackSystem entityId "tyreTrackSystem"
---@param trackId integer "trackId"
function cutTrack(tyreTrackSystem, trackId) end

--- Disable single step script execution

function debugDisableSingleStep() end

--- Enable single step script execution

function debugEnableSingleStep() end

--- Delete Entity/Object
---@param objectId entityId "objectId"
function delete(objectId) end

--- 
---@param foliageBendingSystemId entityId
---@param objectId integer "id of the bending object, such as returned by createFoliageBendingRectangle"
function destroyFoliageBendingObject(foliageBendingSystemId, objectId) end

--- Destroys a tire track
---@param tyreTrackSystem entityId "tyreTrackSystem"
---@param trackId integer "trackId"
function destroyTrack(tyreTrackSystem, trackId) end

--- Disable animation track
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
function disableAnimTrack(characterSetId, trackId) end

--- Render an arrow. Only use for debug rendering
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param dirX number "dirX"
---@param dirY number "dirY"
---@param dirZ number "dirZ"
---@param tangX number "tangX"
---@param tangY number "tangY"
---@param tangZ number "tangZ"
---@param r number "r"
---@param g number "g"
---@param b number "b"
---@param solid boolean|nil "[optional] true: depth is checked and point can be hidden behind other meshes; false: always rendered on top (default=true)"
function drawDebugArrow(x, y, z, dirX, dirY, dirZ, tangX, tangY, tangZ, r, g, b, solid) end

--- Render a line. Only use for debug rendering
---@param x0 number "x0"
---@param y0 number "y0"
---@param z0 number "z0"
---@param r0 number "r0"
---@param g0 number "g0"
---@param b0 number "b0"
---@param x1 number "x1"
---@param y1 number "y1"
---@param z1 number "z1"
---@param r1 number "r1"
---@param g1 number "g1"
---@param b1 number "b1"
---@param solid boolean|nil "[optional] true: depth is checked and point can be hidden behind other meshes; false: always rendered on top (default=true)"
function drawDebugLine(x0, y0, z0, r0, g0, b0, x1, y1, z1, r1, g1, b1, solid) end

--- Render a point. Only use for debug rendering
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param r number "r"
---@param g number "g"
---@param b number "b"
---@param a number "a"
---@param solid boolean|nil "[optional] true: depth is checked and point can be hidden behind other meshes; false: always rendered on top (default=true)"
function drawDebugPoint(x, y, z, r, g, b, a, solid) end

--- Enable/disable full screen dimming
---@param enableDimming boolean "enable/disable dimming"
function drawDebugSetDimmingState(enableDimming) end

--- Enable animation track
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
function enableAnimTrack(characterSetId, trackId) end

--- Erase segments inside parallelogram
---@param tyreTrackSystem entityId "tyreTrackSystem"
---@param startWorldX number "startWorldX"
---@param startWorldZ number "startWorldZ"
---@param widthWorldX number "widthWorldX"
---@param widthWorldZ number "widthWorldZ"
---@param heightWorldX number "heightWorldX"
---@param heightWorldZ number "heightWorldZ"
function eraseParallelogram(tyreTrackSystem, startWorldX, startWorldZ, widthWorldX, widthWorldZ, heightWorldX, heightWorldZ) end

--- Erase segments inside polygonal area
---@param tyreTrackSystem entityId "tyreTrackSystem"
---@param vertexPositions floatArray "an array x1,z1, x2,z2, ... defining the vertices of the polygon"
function erasePolygon(tyreTrackSystem, vertexPositions) end

--- Export all script created notes to file
---@param filePath string
function exportNoteNodes(filePath) end

--- Add material/volume to a fill plane
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param dTvolume number "dTvolume"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param dx1 number "dx1"
---@param dy1 number "dy1"
---@param dz1 number "dz1"
---@param dx2 number "dx2"
---@param dy2 number "dy2"
---@param dz2 number "dz2"
function fillPlaneAdd(fillPlaneShapeId, dTvolume, x, y, z, dx1, dy1, dz1, dx2, dy2, dz2) end

--- Finalize terrain fill layers in given terrain
---@param terrainId entityId "terrainId"
function finalizeTerrainFillLayers(terrainId) end

--- find nearest polyline
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param x number "x"
---@param z number "z"
---@return polyLineId integer "polyLineId"
function findPolyline(fillPlaneShapeId, x, z) end

--- Get all shared I3D file request ids
---@return ids intArray "array of request ids"
function getAllSharedI3DFileRequestIds() end

--- Get all streaming I3D file request ids
---@return ids intArray "array of request ids"
function getAllStreamI3DFileRequestIds() end

--- Get angular damping
---@param transformId entityId "transformId"
---@return angularDamping float "angularDamping"
function getAngularDamping(transformId) end

--- Get angular velocity of transform object
---@param transformId entityId "transformId"
---@return velocityX float "velocityX"
function getAngularVelocity(transformId) end

--- Get animation character set id
---@param objectId entityId "objectId"
---@return characterSetId integer "characterSetId"
function getAnimCharacterSet(objectId) end

--- Get the duration of the clip at the given index
---@param characterSetId entityId "characterSetId"
---@param index integer "index"
---@return duration float "duration"
function getAnimClipDuration(characterSetId, index) end

--- Return the index of the clip with the given name
---@param characterSetId entityId "characterSetId"
---@param clipName string "clipName"
---@return index integer "index"
function getAnimClipIndex(characterSetId, clipName) end

--- Get the name of the clip at the given index
---@param characterSetId entityId
---@param index integer
---@return name string
function getAnimClipName(characterSetId, index) end

--- Get number of clips
---@param characterSetId entityId "characterSetId"
---@return numClips integer "numClips"
function getAnimNumOfClips(characterSetId) end

--- Get animation track assigned clip index number
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@return clipIndex integer "clipIndex"
function getAnimTrackAssignedClip(characterSetId, trackId) end

--- Get animation track blend weight
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@return weight float "weight"
function getAnimTrackBlendWeight(characterSetId, trackId) end

--- Get animation track time
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@return time float "time"
function getAnimTrackTime(characterSetId, trackId) end

--- Get if audio source has auto play enabled
---@param audioSource entityId "audioSource"
---@return autoPlayEnabled boolean
function getAudioSourceAutoPlay(audioSource) end

--- Gets inner range of audio source. If the camera is further than this distance volume will start to decrease
---@param audioSource entityId "audioSource"
---@return range float "inner range (fade start)"
function getAudioSourceInnerRange(audioSource) end

--- Get the AudioSource's max retrigger delay
---@param audioSource entityId "audioSource"
---@return maxRetriggerDelay float "maxRetriggerDelay"
function getAudioSourceMaxRetriggerDelay(audioSource) end

--- Get the AudioSource's min retrigger delay
---@param audioSource entityId "audioSource"
---@return minRetriggerDelay float "minRetriggerDelay"
function getAudioSourceMinRetriggerDelay(audioSource) end

--- Get audio source priority.
---@param audioSource entityId "audioSource"
---@return priority integer
function getAudioSourcePriority(audioSource) end

--- Get the AudioSource's random playback state
---@param audioSource entityId "audioSource"
---@return randomPlayback boolean "randomPlayback"
function getAudioSourceRandomPlayback(audioSource) end

--- Gets (outer) range of audio source
---@param audioSource entityId "audioSource"
---@return range float "outer range (fade end)"
function getAudioSourceRange(audioSource) end

--- Gets the sample id of an audio source
---@param audioSource entityId "audioSource"
---@return sampleId integer "sampleId"
function getAudioSourceSample(audioSource) end

--- Get the AudioSource's sample element's probability
---@param audioSource entityId "audioSource"
---@param index integer "index"
---@return probability float "probability"
function getAudioSourceSampleElementProbability(audioSource, index) end

--- Get the inaudible behavior of the sound. By default, if a sound is inaudible, it's paused, and will resume when it becomes audible again.
---@param audioSource entityId "audioSource"
---@return tickIfInaudible boolean
function getAudioSourceTickInaudible(audioSource) end

--- Get currently active camera
---@return cameraId entityId "cameraId"
function getCamera() end

--- Get can render unicode
---@param unicode integer "unicode"
---@return canRender boolean "canRender"
function getCanRenderUnicode(unicode) end

--- Get character controller collision flags
---@param characterIndex integer "characterIndex"
---@return side boolean "side"
function getCCTCollisionFlags(characterIndex) end

--- Get character height
---@param characterIndex integer "characterIndex"
---@return height float "height"
function getCCTHeight(characterIndex) end

--- Get center of mass (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@return x float "x"
function getCenterOfMass(transformId) end

--- Get first child node matching given name
---@param objectId entityId "objectId"
---@param childName string "childName"
---@return childId entityId "entityId of first child with the given name, 0 if no child matches"
function getChild(objectId, childName) end

--- Get child id at given index. Indices start at 0.
---@param objectId entityId "objectId"
---@param index integer "index "
---@return childId entityId "childId"
function getChildAt(objectId, index) end

--- Get child index of given transform relative to its parent. Indices starting at 0
---@param objectId entityId "objectId"
---@return childIndex integer "childIndex, -1 if transform has no parent"
function getChildIndex(objectId) end

--- Get object clip distance
---@param objectId entityId "objectId"
---@return distance float "distance"
function getClipDistance(objectId) end

--- Get clip distance respecting LODs, ignoring view distance coefficients
---@param objectId entityId "objectId"
---@return minDist float "minDist"
function getClipDistancesWithLOD(objectId) end

--- Get closest world space position and time on spline to given world space position
---@param shapeId integer
---@param worldX number
---@param worldY number
---@param worldZ number
---@param eps number "acceptable world space error"
---@return worldX float
function getClosestSplinePosition(shapeId, worldX, worldY, worldZ, eps) end

--- Get collision filter
---@param transformId entityId "transformId"
---@return group integer "group"
function getCollisionFilter(transformId) end

--- Get collision filter group
---@param transformId entityId "transformId"
---@return group integer "group"
function getCollisionFilterGroup(transformId) end

--- Get collision filter mask
---@param transformId entityId "transformId"
---@return mask integer "mask"
function getCollisionFilterMask(transformId) end

--- Get boolean value
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param parameterId integer "parameterId"
---@return value boolean "value"
function getConditionalAnimationBoolValue(conditionalAnimationEntityId, parameterId) end

--- Get float value
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param parameterId integer "parameterId"
---@return value float "value"
function getConditionalAnimationFloatValue(conditionalAnimationEntityId, parameterId) end

--- Get time
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@return time float
function getConditionalAnimationTime(conditionalAnimationEntityId) end

--- Get current master volume
---@return volume float "volume"
function getCurrentMasterVolume() end

--- Get associated transform group (foliage or terrain fill) from a given dataplane
---@param dataPlaneId entityId "dataPlaneId"
---@return tgId integer "transform group ID"
function getDataPlaneAssociatedTransformGroup(dataPlaneId) end

--- Get density
---@param transformId entityId "transformId"
---@return density float "density"
function getDensity(transformId) end

--- Get density at world position
---@param dataPlaneId entityId "dataPlaneId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return density integer "density"
function getDensityAtWorldPos(dataPlaneId, x, y, z) end

--- Get height of the density map at the world position
---@param fillDataPlaneId entityId "fillDataPlaneId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return height float
function getDensityHeightAtWorldPos(fillDataPlaneId, x, y, z) end

--- Get density map file name
---@param terrainDataPlane entityId "terrainDataPlane"
---@return mapSize integer "mapSize"
function getDensityMapSize(terrainDataPlane) end

--- 
---@param densityMapSyncerId entityId
---@param densityMapId entityId
---@param worldX number "world X coordinate"
---@param worldZ number "world Z coordinate"
---@return cellX integer "cell index"
function getDensityMapSyncerCellIndicesAtWorldPosition(densityMapSyncerId, densityMapId, worldX, worldZ) end

--- Get normal of the density map at the world position
---@param fillDataPlaneId entityId "fillDataPlaneId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return nx float "normal x"
function getDensityNormalAtWorldPos(fillDataPlaneId, x, y, z) end

--- Get all density states at world position
---@param dataPlaneId entityId "dataPlaneId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return states integer "density states (this is all the density map bits excluding the type index)"
function getDensityStatesAtWorldPos(dataPlaneId, x, y, z) end

--- Get density type index at world position
---@param dataPlaneId entityId "dataPlaneId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return typeIndex integer "density type index"
function getDensityTypeIndexAtWorldPos(dataPlaneId, x, y, z) end

--- Get effective clip distance respecting LODs and current view distance coefficients
---@param objectId entityId "objectId"
---@return minDist float "minDist"
function getEffectiveClipDistancesWithLOD(objectId) end

--- Get effective transform object visibility respecting parent hierarchy
---@param transformId entityId "transformId"
---@return visibility boolean "true if node and all its parent nodes are visible"
function getEffectiveVisibility(transformId) end

--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return countScale float "countScale"
function getEmitCountScale(particleSystemId) end

--- Get emitter starting time.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return emitStartTime float "emitStartTime"
function getEmitStartTime(particleSystemId) end

--- Get emitter stop time.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return emitStopTime float "emitStopTime"
function getEmitStopTime(particleSystemId) end

--- Returns the emitter shape of the particle system
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return shapeId integer "shapeId"
function getEmitterShape(particleSystemId) end

--- Returns the emitter shape velocity scale of the particle system
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return shape float "velocity scale"
function getEmitterShapeVelocityScale(particleSystemId) end

--- Get camera far clip distance
---@param cameraId entityId "cameraId"
---@return farClip float "farClip"
function getFarClip(cameraId) end

--- Get the fill plane height at a specific position
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param x number "x"
---@param z number "z"
---@return height float "height"
function getFillPlaneHeightAtLocalPos(fillPlaneShapeId, x, z) end

--- 
---@param foliageBendingSystemId entityId
---@param rectangleId integer
---@return minX float
function getFoliageBendingRectangleAttributes(foliageBendingSystemId, rectangleId) end

--- Get foliage transform group from foliage (dataplane) name
---@param terrainId entityId "terrainId"
---@param detailName string "detailName"
---@return foliageTgId integer "foliageTgId"
function getFoliageTransformGroupIdByFoliageName(terrainId, detailName) end

--- Get the vertical field of view angle
---@param cameraId entityId "id of the camera"
---@return fovY float "field of view angle (radian)"
function getFovY(cameraId) end

--- Get joystick/gamepad axis label
---@param axisNumber integer "axisNumber"
---@param gamepadIndex integer "gamepadIndex"
---@return axisLabel string "axisLabel"
function getGamepadAxisLabel(axisNumber, gamepadIndex) end

--- Get joystick/gamepad button label
---@param buttonNumber integer "buttonNumber"
---@param gamepadIndex integer "gamepadIndex"
---@return buttonLabel string "buttonLabel"
function getGamepadButtonLabel(buttonNumber, gamepadIndex) end

--- Get name of joystick/gamepad
---@param gamepadIndex integer "gamepadIndex"
---@return gamepadName string "gamepadName"
function getGamepadName(gamepadIndex) end

--- Get shape geometry id
---@param shapeId entityId "shapeId"
---@return geometryId integer "geometryId"
function getGeometry(shapeId) end

--- Get has class id
---@param objectId entityId "objectId"
---@param classId integer "one of enum ClassIds"
---@return hasClassId boolean "hasClassId"
function getHasClassId(objectId, classId) end

--- Get if transform group has physics collision
---@param transformId entityId
---@return hasCollision boolean
function getHasCollision(transformId) end

--- Get has shader parameter
---@param shapeId entityId
---@param parameterName string
---@param materialIndex integer|nil "material index or a negative value to set to all materials [optional, default=-1]"
---@return hasParam boolean "hasParam"
function getHasShaderParameter(shapeId, parameterName, materialIndex) end

--- Get existence of a touchpad - this is dynamic (e.g. Switch has touchpad only when not docked)
---@return hasTouchpad boolean "true iff the device has an active touchpad"
function getHasTouchpad() end

--- Get has trigger flag
---@param triggerNodeId entityId
---@return boolean
function getHasTrigger(triggerNodeId) end

--- Get inactive window volume
---@return volume float "volume"
function getInactiveWindowAudioVolume() end

--- Get joystick/gamepad axis value
---@param axisNumber integer "axisNumber"
---@param gamepadIndex integer "gamepadIndex"
---@return axisValue float "axisValue"
function getInputAxis(axisNumber, gamepadIndex) end

--- Get joystick/gamepad button value
---@param buttonNumber integer "buttonNumber"
---@param gamepadIndex integer "gamepadIndex"
---@return buttonValue float "buttonValue"
function getInputButton(buttonNumber, gamepadIndex) end

--- Get if transform is added to physics world (may not immediately return true after addToPhysics due to async effects)
---@param transformId entityId "transformId"
---@return isAddedToPhysics boolean "isAddedToPhysics"
function getIsAddedToPhysics(transformId) end

--- Get if rigid body transform is a compound
---@param transformId entityId "transformId"
---@return isCompound boolean "isCompound"
function getIsCompound(transformId) end

--- Get if rigid body transform is a compound child
---@param transformId entityId "transformId"
---@return isCompound boolean "isCompound"
function getIsCompoundChild(transformId) end

--- Get transform object locked group flag
---@param transformId entityId "transformId"
---@return locked boolean "group locked group"
function getIsLockedGroup(transformId) end

--- Get if transform group is LOD transform group
---@param transformId entityId
---@return trueParam boolean "if transformId is LOD transform group, false otherwise"
function getIsLODTransformGroup(transformId) end

--- Get if shape is non-renderable
---@param shapeId entityId "shapeId"
---@return isNonRenderable boolean
function getIsNonRenderable(shapeId) end

--- Get if camera is in orthographic mode
---@param cameraId entityId "cameraId"
---@return isOrthographic boolean
function getIsOrthographic(cameraId) end

--- Checks if given spline has closed form
---@param shapeId entityId "shapeId"
---@return isClosed boolean "isClosed"
function getIsSplineClosed(shapeId) end

--- Get levenshtein distance
---@param value1 string "value1"
---@param value2 string "value2"
---@return distance integer "distance"
function getLevenshteinDistance(value1, value2) end

--- Returns whether the light source casts a shadow or not.
---@param lightId entityId "lightId"
---@return doesCastShadowMap boolean "doesCastShadowMap"
function getLightCastingShadowMap(lightId) end

--- 
---@param lightId entityId
---@return cone float "angle in radian"
function getLightConeAngle(lightId) end

--- 
---@param lightId entityId
---@return cone float "angle in radian"
function getLightConeAngleFromIESProfile(lightId) end

--- 
---@param lightId entityId
---@return dropoff float
function getLightDropOff(lightId) end

--- 
---@param lightId entityId
---@return ___ies_ string "filepath "
function getLightIESProfile(lightId) end

--- Get range of a light
---@param lightId entityId "lightId"
---@return range float "range"
function getLightRange(lightId) end

--- 
---@param lightId entityId
---@return scattering float "cone angle"
function getLightScatteringConeAngle(lightId) end

--- 
---@param lightId entityId
---@return dirX float
function getLightScatteringDirection(lightId) end

--- 
---@param lightId entityId
---@return scattering float "intensity"
function getLightScatteringIntensity(lightId) end

--- Gets shadow priority (float value) for the given shadow light. Higher value means higher priority (will be picked before lower priority lights when too many shadows are on the screen).
---@param lightId entityId "id of the light source"
---@return shadow shadowPriority "priority value of the light source"
function getLightShadowPriority(lightId) end

--- Gets soft shadow depth bias factor of light source. The bias factor is multiplied with the depth bias of the light source (e.g. depth bias = 0.0001f, bias factor = 2.0f -> depth bias is 0.0002f). They are separated so you can still have the normal bias for PCF shadows (when soft shadows are disabled), which generally
--- need a smaller bias.
---@param lightId entityId "id of the light source"
---@return depth softShadowBias "bias factor for soft shadows used by the light source"
function getLightSoftShadowDepthBiasFactor(lightId) end

--- Gets soft shadow light distance for directional lights (it's fake, fixed distance from each pixel). Ignored by spot lights (though it will still return a value)
---@param lightId entityId "id of the light source"
---@return light softShadowDistance "distance used for the soft shadow"
function getLightSoftShadowDistance(lightId) end

--- Gets soft shadow size. This is essentially the size of the virtual/imagined light source that is casting the soft shadow (for directional lights, instead of an infinitely far away sun, it's a fake square light source). The size of the shadow on the floor is then a function of this size, the light source distance from the ground,
--- and the distance of shadow blocker to the ground. For dir lights, the light source distance is fixed at a fake distance, and can be set with another scripting command (setLightSoftShadowDistance).
---@param lightId entityId "id of the light source"
---@return soft softShadowSize "shadow light size"
function getLightSoftShadowSize(lightId) end

--- Get light type
---@param lightId entityId "lightId"
---@return lightType integer "one of enum LightType"
function getLightType(lightId) end

--- 
---@param lightId entityId
---@return usesLightScattering boolean
function getLightUseLightScattering(lightId) end

--- Get linear damping
---@param transformId entityId "transformId"
---@return linearDamping float "linearDamping"
function getLinearDamping(transformId) end

--- Get linear velocity of transform object (only for kinematical and dynamic physics objects)
---@param transformId entityId "transformId"
---@return velocityX float "velocityX"
function getLinearVelocity(transformId) end

--- Get closest world space position and time on spline to given world space position
---@param shapeId integer "shapeId"
---@param time number
---@param timeRange number "(searches in -/+ 0.5*range)"
---@param worldX number
---@param worldY number
---@param worldZ number
---@param eps number "epsilon value in meters used for matching precision. Minimum value 0.001"
---@return worldX float
function getLocalClosestSplinePosition(shapeId, time, timeRange, worldX, worldY, worldZ, eps) end

--- Get local linear velocity of transform object (only for kinematical and dynamic physics objects)
---@param transformId entityId "transformId"
---@return velocityX float "velocityX"
function getLocalLinearVelocity(transformId) end

--- Returns the LOD transform group that this transform group belongs to
---@param transformId entityId "transformId is either a shape, an audio source or a light source"
---@return lodTransformId integer
function getLODTransformGroup(transformId) end

--- Get mass
---@param transformId entityId "transformId"
---@return mass float "mass in tons"
function getMass(transformId) end

--- Get master volume
---@return volume float "volume"
function getMasterVolume() end

--- Get material by index
---@param shapeId entityId "shapeId"
---@param materialIndex integer "materialIndex"
---@return materialId integer "materialId"
function getMaterial(shapeId, materialIndex) end

--- 
---@param materialId entityId
---@return customShaderFilename string "filename of the custom shader"
function getMaterialCustomShaderFilename(materialId) end

--- 
---@param materialId entityId
---@return customShaderVariation string "name of the custom shader variation"
function getMaterialCustomShaderVariation(materialId) end

--- 
---@param materialId entityId
---@return filename string "filename"
function getMaterialDiffuseMapFilename(materialId) end

--- 
---@param materialId entityId
---@return hasAlphaChannel boolean
function getMaterialDiffuseMapHasAlpha(materialId) end

--- 
---@param materialId entityId
---@return filename string "filename"
function getMaterialEmissiveMapFilename(materialId) end

--- 
---@param materialId entityId
---@return hasAlphaChannel boolean
function getMaterialEmissiveMapHasAlpha(materialId) end

--- 
---@param materialId entityId
---@return filename string "filename"
function getMaterialGlossMapFilename(materialId) end

--- 
---@param materialId entityId
---@return hasRefractionMap boolean
function getMaterialHasRefractionMap(materialId) end

--- 
---@param materialId entityId
---@return isAlphaBlended boolean
function getMaterialIsAlphaBlended(materialId) end

--- 
---@param materialId entityId
---@return isAlphaTested boolean
function getMaterialIsAlphaTested(materialId) end

--- 
---@param materialId entityId
---@return filename string "filename"
function getMaterialNormalMapFilename(materialId) end

--- 
---@param materialId entityId
---@return normalFormat integer "format of enum NormalMapFormat"
function getMaterialNormalMapFormat(materialId) end

--- 
---@param materialId entityId
---@return reflectionMapScaling float "scaling of reflection map. 0 if no reflection map."
function getMaterialReflectionMapScaling(materialId) end

--- Returns the shapes slot name for the material at the given index.
---@param shapeId entityId "shapeId"
---@param materialIndex integer "material index of shape, starting at 0"
---@return slotName string "slotName"
function getMaterialSlotName(shapeId, materialIndex) end

--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return maxNumParticles integer "maximum number of active particles"
function getMaxNumOfParticles(particleSystemId) end

--- Gets ID of the shadow settings light used in a given group of light sources with merged shadows. Will return id 0 if the light source doesn't have any settings light, or if its not part of a merged shadow group.
---@param lightId entityId "id of one of the light sources in a given merged shadow group"
---@return id settingsLightId "of the shadow settings light (or 0 if there isn't one assigned to the merged shadow group)"
function getMergedShadowSettingsLight(lightId) end

--- Get minimum clip distance
---@param objectId entityId "objectId"
---@return minDist float "minDist"
function getMinClipDistance(objectId) end

--- Get motor rotation speed
---@param transformId entityId "transformId"
---@return motorRotSpeed float "motorRotSpeed"
function getMotorRotationSpeed(transformId) end

--- Get motor torque
---@param transformId entityId "transformId"
---@return float
function getMotorTorque(transformId) end

--- Get entity name
---@param entityId entityId "entityId"
---@return entityName string "entityName"
function getName(entityId) end

--- Get camera near clip distance
---@param cameraId entityId "cameraId"
---@return nearClip float "nearClip"
function getNearClip(cameraId) end

--- Get note node text
---@param noteId entityId "Id of the note node"
---@return text string
function getNoteNodeText(noteId) end

--- Get the number of sample elements for an AudioSource
---@param audioSource entityId "audioSource"
---@return count integer "count"
function getNumOfAudioSourceSampleElements(audioSource) end

--- Get number of children
---@param objectId entityId "objectId"
---@return numOfChildren integer "numOfChildren"
function getNumOfChildren(objectId) end

--- Get number of joysticks/gamepads
---@return numOfGamepads integer "numOfGamepads"
function getNumOfGamepads() end

--- Get number of materials
---@param shapeId entityId "shapeId"
---@return numMaterials integer "numMaterials"
function getNumOfMaterials(shapeId) end

--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return emittedParticlesPerMs float "emittedParticlesPerMs"
function getNumOfParticlesToEmitPerMs(particleSystemId) end

--- 
---@param shapeId entityId "shapeId"
---@return numOfBones integer
function getNumOfShapeBones(shapeId) end

--- Gets the number of shared I3d files
---@return fileCount integer "fileCount"
function getNumOfSharedI3DFiles() end

--- Get number of user attributes
---@param objectId entityId "objectId"
---@return numAttributes integer "numAttributes"
function getNumOfUserAttributes(objectId) end

--- Get number of spline attributes
---@param shapeId integer
---@return numAttributes integer
function getNumSplineAttributes(shapeId) end

--- Get object mask
---@param objectId entityId "objectId"
---@return mask integer "mask"
function getObjectMask(objectId) end

--- Get camera orthographic height
---@param cameraId entityId "cameraId"
---@return orthographicHeight float
function getOrthographicHeight(cameraId) end

--- Get parent id
---@param objectId entityId "objectId"
---@return parentId entityId "parentId, 0 if transform has no parent e.g. node is not linked or rootNode"
function getParent(objectId) end

--- Get particle system average speed.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return normalSpeed float "normalSpeed"
function getParticleSystemAverageSpeed(particleSystemId) end

--- Get particle system life span.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return lifeSpan float "lifeSpan"
function getParticleSystemLifespan(particleSystemId) end

--- Get particle system speed
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@return speed float "speed"
function getParticleSystemSpeed(particleSystemId) end

--- Get camera projection offset
---@param cameraId entityId "cameraId"
---@return x float "x"
function getProjectionOffset(cameraId) end

--- Get quaternion in local space
---@param objectId entityId "objectId"
---@return x float "x"
function getQuaternion(objectId) end

--- Get rigid body AABB
---@param transformId entityId "transformId"
---@return minX float
function getRigidBodyAABB(transformId) end

--- Get rigid body type
---@param transformId entityId "transformId"
---@return one integer "of RIGID_BODY_TYPE"
function getRigidBodyType(transformId) end

--- Get root node
---@return transformId entityId "transformId"
function getRootNode() end

--- Get local space rotation of a transform object (relative to its parent)
---@param transformId entityId "transformId"
---@return x float "local x rotation in radians"
function getRotation(transformId) end

--- Get sample duration
---@param sampleId entityId "sampleId"
---@return duration float "duration"
function getSampleDuration(sampleId) end

--- Get the loop synthesis sample load factor currently played
---@param sampleId entityId "sampleId"
---@return loadFactor float "loadFactor normalized between [-1;1]"
function getSampleLoopSynthesisLoadFactor(sampleId) end

--- Get the loop synthesis sample max RPM
---@param sampleId entityId "sampleId"
---@return max float "RPM"
function getSampleLoopSynthesisMaxRPM(sampleId) end

--- Get the loop synthesis sample min RPM
---@param sampleId entityId "sampleId"
---@return min float "RPM"
function getSampleLoopSynthesisMinRPM(sampleId) end

--- Get the loop synthesis sample RPM currently played
---@param sampleId entityId "sampleId"
---@param useNormalizedValue boolean "If true, return normalized value between [0;1]; else return real range value."
---@return rpm float "rpm"
function getSampleLoopSynthesisRPM(sampleId, useNormalizedValue) end

--- Get the loop synthesis sample start sound duration
---@param sampleId entityId "sampleId"
---@return startDuration float "duration [ms] of the loop synthesis start sound."
function getSampleLoopSynthesisStartDuration(sampleId) end

--- Get the loop synthesis sample stop sound duration
---@param sampleId entityId "sampleId"
---@return stopDuration float "duration [ms] of the loop synthesis stop sound. Shoud be passed to the stopSample delay to make sure the stop sound can be played fully."
function getSampleLoopSynthesisStopDuration(sampleId) end

--- Get the loop synthesis sample target load factor
---@param sampleId entityId "sampleId"
---@return loadFactor float "loadFactor normalized between [-1;1]"
function getSampleLoopSynthesisTargetLoadFactor(sampleId) end

--- Get the loop synthesis sample target RPM
---@param sampleId entityId "sampleId"
---@param useNormalizedValue boolean "If true, return normalized value between [0;1]; else return real range value."
---@return rpm float "rpm"
function getSampleLoopSynthesisTargetRPM(sampleId, useNormalizedValue) end

--- Set sample pitch
---@param sampleId entityId "sampleId"
---@return pitch float "pitch"
function getSamplePitch(sampleId) end

--- Get sample play offset
---@param sampleId entityId "sampleId"
---@return offset float "offset"
function getSamplePlayOffset(sampleId) end

--- Get sample left play time
---@param sampleId entityId "sampleId"
---@return leftPlayTime float
function getSamplePlayTimeLeft(sampleId) end

--- Get velocity of a sample object
---@param sampleId entityId "sampleId"
---@return x float "x"
function getSampleVelocity(sampleId) end

--- Get sample volume
---@param sampleId entityId "sampleId"
---@return volume float "volume"
function getSampleVolume(sampleId) end

--- Get scale of a transform object
---@param transformId entityId "transformId"
---@return x float "x"
function getScale(transformId) end

--- Get shader parameter
---@param shapeId entityId "shapeId"
---@param parameterName string "parameterName"
---@param materialIndex integer|nil "[optional, default=0]"
---@return x float "x"
function getShaderParameter(shapeId, parameterName, materialIndex) end

--- Gets the fake extra terrain height (for terrain under the water) that is used in the simulation, in order to make the water deeper than it really is.
---@param shallowWaterSimulation entityId "simulation/id of the shallow water sim to retrieve the texture from"
---@return height float "extra height added to the water under the rest level"
function getShallowWaterSimulationFakeExtraDepth(shallowWaterSimulation) end

--- Gets foam accumulation rate (determines how much and how quickly foam spawns in turbulent waters).
---@param shallowWaterSimulation entityId "simulation/id of the shallow water sim to retrieve the texture from"
---@return foam float "accumulation rate"
function getShallowWaterSimulationFoamAccumulationRate(shallowWaterSimulation) end

--- Gets foam decay rate (how quickly foam decays after it has spawned).
---@param shallowWaterSimulation entityId "simulation/id of the shallow water sim to retrieve the texture from"
---@return foam float "decay rate"
function getShallowWaterSimulationFoamDecayRate(shallowWaterSimulation) end

--- Returns output texture of the simulation, which defines the total height of the water surface at every point
---@param shallowWaterSimulation entityId "simulation/id of the shallow water sim to retrieve the texture from"
---@return textureId integer "id of the output texture"
function getShallowWaterSimulationOutputTexture(shallowWaterSimulation) end

--- Returns u (aka x) component of velocity output texture of the simulation, which defines the u component of velocity of the simulation at every point.
---@param shallowWaterSimulation entityId "simulation/id of the shallow water sim to retrieve the texture from"
---@return textureId integer "id of the velocity u texture"
function getShallowWaterSimulationOutputVelocityUTexture(shallowWaterSimulation) end

--- Returns v (aka y) component of velocity output texture of the simulation, which defines the v component of velocity of the simulation at every point.
---@param shallowWaterSimulation entityId "simulation/id of the shallow water sim to retrieve the texture from"
---@return textureId integer "id of the velocity u texture"
function getShallowWaterSimulationOutputVelocityVTexture(shallowWaterSimulation) end

--- 
---@param shapeId entityId
---@param boneIndex integer
---@return boneId integer
function getShapeBone(shapeId, boneIndex) end

--- Returns bounding sphere of shape
---@param shapeId entityId "shapeId"
---@return localPosX float
function getShapeBoundingSphere(shapeId) end

--- Get shape build nav mesh mask
---@param shapeId entityId
---@return mask integer
function getShapeBuildNavMeshMask(shapeId) end

--- Returns bounding sphere of the shape's geometry.
---@param shapeId entityId "shapeId"
---@return localPosX float
function getShapeGeometryBoundingSphere(shapeId) end

--- Get if shape is marked as CPU mesh
---@param shapeId entityId "shapeId"
---@return isCPUMesh boolean
function getShapeIsCPUMesh(shapeId) end

--- Get if shape is skinned
---@param shapeId entityId "shapeId"
---@return isSkinned boolean
function getShapeIsSkinned(shapeId) end

--- Returns world space bounding sphere of shape
---@param shapeId entityId "shapeId"
---@return worldPosX float
function getShapeWorldBoundingSphere(shapeId) end

--- Get shared I3D file progress information
---@param requestId integer "request id from streamSharedI3DFile"
---@return progress string "a string describing the progress"
function getSharedI3DFileProgressInfo(requestId) end

--- Gets the number references a shared I3D file has
---@param filename string
---@return refCount integer "Number of references for the shared i3d. Smaller than 0 if file is not loaded or loading. Can be 0 when the file is still loaded (e.g. due to a releaseSharedI3DFile call with autoDelete = false)."
function getSharedI3DFileRefCount(filename) end

--- Get the spline attribute value at given t
---@param shapeId integer
---@param attributeIndex integer "attribute index (0 based)"
---@param t number
---@return value float
function getSplineAttribute(shapeId, attributeIndex, t) end

--- Get the spline attribute value at given CV
---@param shapeId integer
---@param attributeIndex integer "attribute index (0 based)"
---@param CVIndex integer
---@return value float
function getSplineAttributeAtCV(shapeId, attributeIndex, CVIndex) end

--- Get the index of a spline attribute
---@param shapeId integer
---@param attributeName string
---@return attributeIndex integer
function getSplineAttributeIndex(shapeId, attributeName) end

--- Get spline attribute name
---@param shapeId integer
---@param attributeIndex integer "attribute index (0 based)"
---@return attributeName string
function getSplineAttributeName(shapeId, attributeIndex) end

--- Get spline curvature
---@param shapeId entityId "shapeId"
---@param time number "time"
---@return curvature float "curvature"
function getSplineCurvature(shapeId, time) end

--- Get spline control vertex
---@param shapeId entityId "shapeId"
---@param index integer "index starting at 0"
---@return x float "worldspace x"
function getSplineCV(shapeId, index) end

--- Get spline direction
---@param shapeId entityId "shapeId"
---@param time number "time"
---@return dirX float "dirX"
function getSplineDirection(shapeId, time) end

--- Get spline edit point
---@param shapeId integer "shapeId"
---@param index integer "The index of edit point to be positioned, starting at 0"
---@return x float "worldspace x"
function getSplineEP(shapeId, index) end

--- Get spline length
---@param shapeId entityId "shapeId"
---@return length float "length"
function getSplineLength(shapeId) end

--- Get number of spline control vertices
---@param shapeId entityId "shapeId"
---@return num integer "num"
function getSplineNumOfCV(shapeId) end

--- Get spline orientation
---@param shapeId entityId "shapeId"
---@param time number "time"
---@param upDirX number "upDirX"
---@param upDirY number "upDirY"
---@param upDirZ number "upDirZ"
---@return rx float "rx"
function getSplineOrientation(shapeId, time, upDirX, upDirY, upDirZ) end

--- Get spline position
---@param shapeId entityId "shapeId"
---@param time number "time"
---@return x float "x"
function getSplinePosition(shapeId, time) end

--- Get world space position and time on spline that has the given world space distance to the position on the spline at the given time
---@param shapeId integer "shapeId"
---@param time number
---@param distance number
---@param positiveTimeOffset boolean "search in positive or negative direction of t"
---@param eps number "epsilon value in meters used for matching precision. Minimum value 0.001"
---@return worldX float
function getSplinePositionWithDistance(shapeId, time, distance, positiveTimeOffset, eps) end

--- Get split type of shape
---@param shapeId entityId "shapeId"
---@return splitType integer "splitType, 0 if shape is not a mesh split shape"
function getSplitType(shapeId) end

--- Get streamed sample volume
---@param streamedSampleId entityId "streamedSampleId"
---@return volume float "volume"
function getStreamedSampleVolume(streamedSampleId) end

--- Get streaming I3D file progress information
---@param requestId integer "request id from streamI3DFile"
---@return progress string "a string describing the progress"
function getStreamI3DFileProgressInfo(requestId) end

--- Get terrain attributes at world pos
---@param terrainId entityId "terrainId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param xComb boolean "xComb"
---@param yComb boolean "yComb"
---@param zComb boolean "zComb"
---@param wComb boolean "wComb"
---@param uComb boolean "uComb"
---@return x float "x"
function getTerrainAttributesAtWorldPos(terrainId, x, y, z, xComb, yComb, zComb, wComb, uComb) end

--- Get terrain data plane by name
---@param terrainId entityId "terrainId"
---@param detailName string "detailName"
---@return detailId integer "detailId"
function getTerrainDataPlaneByName(terrainId, detailName) end

--- Get terrain detail by name
---@param terrainId entityId "terrainId"
---@param detailName string "detailName"
---@return detailId integer "detailId"
function getTerrainDetailByName(terrainId, detailName) end

--- Get terrain detail name
---@param dataPlaneId entityId "dataPlaneId"
---@return viewDistance string "viewDistance"
function getTerrainDetailName(dataPlaneId) end

--- Get number of terrain detail channels
---@param dataPlaneId entityId "dataPlaneId"
---@return numChannels integer "numChannels"
function getTerrainDetailNumChannels(dataPlaneId) end

--- Get density map type ID for terrain detail
---@param dataPlaneId entityId "dataPlaneId"
---@return typeIndex integer "type index"
function getTerrainDetailTypeIndex(dataPlaneId) end

--- Get terrain height at world pos
---@param terrainId entityId "terrainId"
---@param x number "x"
---@param y number "y (not relevant, can be 0)"
---@param z number "z"
---@return height float "height"
function getTerrainHeightAtWorldPos(terrainId, x, y, z) end

--- Get terrain heightmap unit size
---@param terrainId entityId "terrainId"
---@return unitSize float "the unit size for the main terrain heightmap"
function getTerrainHeightmapUnitSize(terrainId) end

--- Get terrain normal at world pos
---@param terrainId entityId "terrainId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return nx float "nx"
function getTerrainNormalAtWorldPos(terrainId, x, y, z) end

--- Get terrain size
---@param terrainId entityId "terrainId"
---@return size float "size"
function getTerrainSize(terrainId) end

--- Get 3D text width
---@param fontSize number "fontSize "
---@param utf8string string "utf8string"
---@return width float "normalized screenspace width of given text at given size"
function getText3DWidth(fontSize, utf8string) end

--- Get text height
---@param fontSize number "fontSize"
---@param utf8string string "utf8string"
---@return textHeight float "textHeight"
function getTextHeight(fontSize, utf8string) end

--- Get text length
---@param fontSize number "fontSize"
---@param utf8string string "utf8string"
---@param maxNumLines integer "maxNumLines"
---@return numChars integer "numChars"
function getTextLength(fontSize, utf8string, maxNumLines) end

--- Get text length
---@param fontSize number "fontSize"
---@param utf8string string "utf8string"
---@param maxWidth number "maxWidth"
---@return numChars integer "numChars"
function getTextLineLength(fontSize, utf8string, maxWidth) end

--- Get text width
---@param fontSize number "fontSize"
---@param utf8string string "utf8string"
---@return textWidth float "textWidth"
function getTextWidth(fontSize, utf8string) end

--- Get spline time of the control vertex with the given index
---@param shapeId integer "shapeId"
---@param index integer "index of the control vertex, starting at 0"
---@return t float "t"
function getTimeAtSplineCV(shapeId, index) end

--- Get translation of a transform object in its local space (relative to parent node)
---@param transformId entityId "transformId"
---@return x float "x translation relative to parent node"
function getTranslation(transformId) end

--- Get user attribute value
---@param objectId entityId
---@param name string
---@return value any? " [optional]"
function getUserAttribute(objectId, name) end

--- Get user attribute value by index starting at 0
---@param objectId entityId "objectId"
---@param attributeIndex integer "attributeIndex starting at 0"
---@return value any? " [optional]"
function getUserAttributeByIndex(objectId, attributeIndex) end

--- Get user attribute value and type
---@param objectId entityId
---@param name string
---@return value any? " [optional]"
function getUserAttributeValueAndType(objectId, name) end

--- Get velocity at local position of transform object
---@param transformId entityId "transformId"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
---@return velocityX float "velocityX"
function getVelocityAtLocalPos(transformId, positionX, positionY, positionZ) end

--- Get velocity at world position of transform object (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
---@return velocityX float "velocityX"
function getVelocityAtWorldPos(transformId, positionX, positionY, positionZ) end

--- Get transform object visibility, ignoring parent hierarchy visibility
---@param transformId entityId "transformId"
---@return visibility boolean "true if node itself is visible"
function getVisibility(transformId) end

--- Get the day of year condition of the visibility condition
---@param objectId entityId "objectId"
---@return dayOfYearStart integer "dayOfYearStart"
function getVisibilityConditionDayOfYear(objectId) end

--- Get the visibility condition entity state
---@param objectId entityId "objectId"
---@return isVisible boolean
function getVisibilityConditionEntityState(objectId) end

--- Get the minute of day condition of the visibility condition
---@param objectId entityId "objectId"
---@return minuteOfDayStart integer "minuteOfDayStart"
function getVisibilityConditionMinuteOfDay(objectId) end

--- Get the render invisible property of the visibility condition
---@param objectId entityId "objectId"
---@return renderInvisible boolean "if true, the object will always be rendered and the custom shader is supposed to change the rendering based on the visibility parameter"
function getVisibilityConditionRenderInvisible(objectId) end

--- Get the viewerspaciality mask condition of the visibility condition
---@param objectId entityId "objectId"
---@return viewerSpacialityRequiredMask integer
function getVisibilityConditionViewerSpacialityMask(objectId) end

--- Get the shader parameter of the visibility condition
---@param objectId entityId "objectId"
---@return shaderVisibilityParam float "shader parameter when condition is met (ie. object is visible)"
function getVisibilityConditionVisibleShaderParameter(objectId) end

--- Get the weather mask condition of the visibility condition
---@param objectId entityId "objectId"
---@return weatherRequiredMask integer
function getVisibilityConditionWeatherMask(objectId) end

--- Get volume, only for dynamic and kinematic shapes, 0 otherwise
---@param transformId entityId "transformId"
---@return volume float "volume in cubic meters"
function getVolume(transformId) end

--- Get wheel shape axle speed
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return axleSpeed float "axleSpeed"
function getWheelShapeAxleSpeed(transformId, wheelShapeIndex) end

--- Get wheel shape contact force
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return contactForce float "contactForce"
function getWheelShapeContactForce(transformId, wheelShapeIndex) end

--- Get wheel shape contact normal
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return x float "x"
function getWheelShapeContactNormal(transformId, wheelShapeIndex) end

--- Get wheel shape contact object
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return wheelShapeContactObject entityId "wheelShapeContactObject"
function getWheelShapeContactObject(transformId, wheelShapeIndex) end

--- Get wheel shape contact point
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return positionX float "positionX"
function getWheelShapeContactPoint(transformId, wheelShapeIndex) end

--- Get wheel shape contact point
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return positionX float "positionX"
function getWheelShapePosition(transformId, wheelShapeIndex) end

--- Get wheel shape slip
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@return wheelShapeSlip float "wheelShapeSlip"
function getWheelShapeSlip(transformId, wheelShapeIndex) end

--- Get quaternion in world space
---@param objectId entityId "objectId"
---@return x float "x"
function getWorldQuaternion(objectId) end

--- Get world rotation of a transform object
---@param transformId entityId "transformId"
---@return x float "x"
function getWorldRotation(transformId) end

--- Get translation of a transform object in world space
---@param transformId entityId "transformId"
---@return x float "x translation"
function getWorldTranslation(transformId) end

--- Returns the name of the attribute of the specified xml element at the given index
---@param xmlId entityId "xmlId"
---@param xmlElementPath string "Path to an xml element"
---@param index integer "Index of the attribute to retrieve the name for"
---@return attributeName string
function getXMLAttributeName(xmlId, xmlElementPath, index) end

--- Get XML file boolean attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@return value boolean "value"
function getXMLBool(xmlId, attributePath) end

--- Returns the name of the specified xml element. Mostly useful when using wildcard paths
---@param xmlId entityId "xmlId"
---@param attributePath string
---@return elementName string
function getXMLElementName(xmlId, attributePath) end

--- Get XML filename
---@param xmlId entityId "xmlId"
---@return xmlFilename string "filename of xml entity (can be empty if not loaded from file)"
function getXMLFilename(xmlId) end

--- Get XML file float attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@return value float "value"
function getXMLFloat(xmlId, attributePath) end

--- Get XML file integer attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@return value integer "value"
function getXMLInt(xmlId, attributePath) end

--- Returns the number of attributes of the element at the given path
---@param xmlId entityId "xmlId"
---@param xmlElementPath string "Path to an xml element"
---@return numOfAttributes integer
function getXMLNumOfAttributes(xmlId, xmlElementPath) end

--- Returns the number of children of a given element
---@param xmlId entityId "xmlId"
---@param xmlElementPath string "Path to an xml element"
---@return numOfChildren integer
function getXMLNumOfChildren(xmlId, xmlElementPath) end

--- Returns the number of equally named elements with the given path (the last element must not have an index, or the index must be 0)
---@param xmlId entityId "xmlId"
---@param xmlElementPath string "Path to an xml element"
---@return numOfElements integer
function getXMLNumOfElements(xmlId, xmlElementPath) end

--- Get XML file string attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@return value string "value"
function getXMLString(xmlId, attributePath) end

--- Get XML file unsigned integer attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@return value integer "value"
function getXMLUInt(xmlId, attributePath) end

--- Returns whether the given light source has a merged shadow (i.e. its shadow is merged with other light sources).
---@param lightId entityId "id of the light source node"
---@return has boolean "merged shadow, yes or no!"
function hasMergedShadow(lightId) end

--- Returns if an XML element or attribute at given path is present.
---@param xmlId entityId "xmlId"
---@param xmlElementPath string "Path to an xml element"
---@return hasElementOrAttribute boolean
function hasXMLProperty(xmlId, xmlElementPath) end

--- Init animations
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param charsetNodeId entityId "character set node id"
---@param xmlFilePath string "xmlFilePath"
---@param baseNodeName string "baseNodeName"
function initConditionalAnimation(conditionalAnimationEntityId, charsetNodeId, xmlFilePath, baseNodeName) end

--- Makes sure the bounding volume of the shape is up to date with the geometry bounding volume.
--- This needs to be called after changing the bounding volume of the geometry if the bounding volume of the shape is not invalidated otherwise (e.g. by moving)
---@param shapeId entityId
function invalidateShapeBoundingVolume(shapeId) end

--- Is clip assigned to animation track
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@return isClipAssigned boolean "isClipAssigned"
function isAnimTrackClipAssigned(characterSetId, trackId) end

--- Is animation track enabled
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@return state boolean "state"
function isAnimTrackEnabled(characterSetId, trackId) end

--- For the given light source, returns whether the merged shadow (for the entire group of lights merged together) is active.
---@param lightId entityId "id of the light source node"
---@return isActive boolean "true - merged shadows is active. false - merged shadow is inactive"
function isMergedShadowActive(lightId) end

--- Is sample playing
---@param sampleId entityId
---@return isPlaying boolean
function isSamplePlaying(sampleId) end

--- Set joint actors
---@param actor1Id entityId "actor1Id"
---@param actor2Id entityId "actor2Id"
function JointConstructor:setActors(actor1Id, actor2Id) end

--- Set joint transforms
---@param jointNode1 entityId "jointNode1"
---@param jointNode2 entityId "jointNode2"
function JointConstructor:setJointTransforms(jointNode1, jointNode2) end

--- Link node to another node
---@param parentNodeId entityId "node to link to"
---@param childNodeId entityId "node to be linked"
---@param index integer|nil "[optional] child index where childNodeId is linked to (default: append after last child)"
function link(parentNodeId, childNodeId, index) end

--- Load I3D file
---@param filename string "filename"
---@param addPhysics boolean|nil "addPhysics [optional]"
---@param callOnCreate boolean|nil "callOnCreate [optional]"
---@param verbose boolean|nil "verbose [optional]"
---@return rootNodeId integer "rootNodeId"
function loadI3DFile(filename, addPhysics, callOnCreate, verbose) end

--- Load sample object
---@param sampleId entityId "sampleId"
---@param sampleFilename string "sampleFilename"
---@param b3DSound boolean "b3DSound"
---@return success boolean "success"
function loadSample(sampleId, sampleFilename, b3DSound) end

--- Load shared I3D file. If another shared stream request is still pending for the same i3d, the call blocks until this request is finished
---@param filename string "filename"
---@param addPhysics boolean|nil "addPhysics [optional]"
---@param callOnCreate boolean|nil "callOnCreate [optional]"
---@param verbose boolean|nil "verbose [optional]"
---@return rootNodeId integer "rootNodeId"
function loadSharedI3DFile(filename, addPhysics, callOnCreate, verbose) end

--- Load streamed sample object
---@param streamedSampleId entityId "streamedSampleId"
---@param sampleFilename string "sampleFilename"
---@return success boolean "success"
function loadStreamedSample(streamedSampleId, sampleFilename) end

--- Load XML file
---@param objectName string "objectName"
---@param filename string "filename"
---@return xmlId integer "xmlId (0 if failed to load)"
function loadXMLFile(objectName, filename) end

--- Load XML file from xml string
---@param objectName string "objectName"
---@param xmlString string "xml string to parse"
---@return xmlId integer "xmlId (0 if failed to load)"
function loadXMLFileFromMemory(objectName, xmlString) end

--- Local space to local space transformation, only direction without translation
---@param transformId entityId "transformId"
---@param targetTransformId entityId "targetTransformId"
---@param dx number "dx"
---@param dy number "dy"
---@param dz number "dz"
---@return dx float "dx"
function localDirectionToLocal(transformId, targetTransformId, dx, dy, dz) end

--- Local space to world space direction transformation
---@param transformId entityId "transformId"
---@param ldx number "local direction x"
---@param ldy number "local direction y"
---@param ldz number "local direction z"
---@return wdx float "world direction x"
function localDirectionToWorld(transformId, ldx, ldy, ldz) end

--- Local space to local space rotation transformation
---@param transformId entityId "transformId"
---@param targetTransformId entityId "targetTransformId"
---@param rx number "x rotation"
---@param ry number "y rotation"
---@param rz number "z rotation"
---@return rx float "x rotation"
function localRotationToLocal(transformId, targetTransformId, rx, ry, rz) end

--- Local space to world space rotation transformation
---@param transformId entityId "transformId"
---@param lrx number "local rotation x"
---@param lry number "local rotation y"
---@param lrz number "local rotation z"
---@return wrx float "world rotation x"
function localRotationToWorld(transformId, lrx, lry, lrz) end

--- Local space to local space transformation
---@param transformId entityId "transformId"
---@param targetTransformId entityId "targetTransformId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return x float "x"
function localToLocal(transformId, targetTransformId, x, y, z) end

--- Local space to world space transformation
---@param transformId entityId "transformId"
---@param lx number "local x translation"
---@param ly number "local y translation"
---@param lz number "local z translation"
---@return wx float "world x translation"
function localToWorld(transformId, lx, ly, lz) end

--- get currently calibrated color in sRGB format and error value.
---@param angle integer "One of the angle values (-15, 15, 25, 45, 75 or 110)"
---@return color floatArray
function MaterialCalibrationSolver:getCurrentSRGB(angle) end

--- Get target color in sRGB format.
---@param angle integer "One of the angle values (-15, 15, 25, 45, 75 or 110)"
---@return color floatArray
function MaterialCalibrationSolver:getTargetSRGB(angle) end

--- Is calibration completed.
---@return isDone boolean
function MaterialCalibrationSolver:isDone() end

--- Execute one iteration of a calibration.
---@return integer
function MaterialCalibrationSolver:iterate() end

--- Prepare everything for calibration process. Use data provided via setters.

function MaterialCalibrationSolver:prepare() end

--- Reset calibrator to original state.

function MaterialCalibrationSolver:reset() end

--- Set which channels will be used in calibration
---@param channelR boolean
---@param channelG boolean
---@param channelB boolean
---@param channelSmoothness boolean
---@param channelMetalness boolean
function MaterialCalibrationSolver:setChannels(channelR, channelG, channelB, channelSmoothness, channelMetalness) end

--- Set which custom parameter channels will be used in calibration
---@param parameterName string
---@param channelR boolean
---@param channelG boolean
---@param channelB boolean
---@param channelA boolean
function MaterialCalibrationSolver:setCustomParameterChannels(parameterName, channelR, channelG, channelB, channelA) end

--- Set value range for custom parameter channel
---@param parameterName string
---@param channelIndex integer
---@param minValue number
---@param maxValue number
function MaterialCalibrationSolver:setCustomParameterChannelValueRange(parameterName, channelIndex, minValue, maxValue) end

--- Set a shape which will be used in calibration. Solver will take first material from that shape
---@param shapeId entityId
function MaterialCalibrationSolver:setShape(shapeId) end

--- Set value range for channel B
---@param minValue number
---@param maxValue number
function MaterialCalibrationSolver:setValueRangeChannelB(minValue, maxValue) end

--- Set value range for channel G
---@param minValue number
---@param maxValue number
function MaterialCalibrationSolver:setValueRangeChannelG(minValue, maxValue) end

--- Set value range for channel metalness
---@param minValue number
---@param maxValue number
function MaterialCalibrationSolver:setValueRangeChannelMetalness(minValue, maxValue) end

--- Set value range for channel R
---@param minValue number
---@param maxValue number
function MaterialCalibrationSolver:setValueRangeChannelR(minValue, maxValue) end

--- Set value range for channel smoothness
---@param minValue number
---@param maxValue number
function MaterialCalibrationSolver:setValueRangeChannelSmoothness(minValue, maxValue) end

--- Euler angle vector rotation
---@param x number "euler angle x axis"
---@param y number "euler angle y axis"
---@param z number "euler angle z axis"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return x float "x"
function mathEulerRotateVector(x, y, z, x, y, z) end

--- Euler angle to quaternion
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return qx float "qx"
function mathEulerToQuaternion(x, y, z) end

--- Quaternion vector rotation
---@param qx number "qx"
---@param qy number "qy"
---@param qz number "qz"
---@param qw number "qw"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@return x float "x"
function mathQuaternionRotateVector(qx, qy, qz, qw, x, y, z) end

--- Quaternion to euler angle
---@param qx number "qx"
---@param qy number "qy"
---@param qz number "qz"
---@param qw number "qw"
---@return x float "x"
function mathQuaternionToEuler(qx, qy, qz, qw) end

--- Merge shadows of multiple light sources and replace them with a approximated and representative match
---@param lightId entityId
---@param lightId2 entityId
---@param lightId3 entityId "optional light id 3"
---@param lightId4 entityId "optional light id 4"
---@param lightId5 entityId "optional light id 5"
---@param lightId6 entityId "optional light id 6"
---@param lightId7 entityId "optional light id 7"
---@param lightId8 entityId "optional light id 8"
---@param lightId9 entityId "optional light id 9"
---@param lightId10 entityId "optional light id 10"
function mergeLightShadows(lightId, lightId2, lightId3, lightId4, lightId5, lightId6, lightId7, lightId8, lightId9, lightId10) end

--- Enqueue character movement
---@param characterIndex integer "characterIndex"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param collisionGroup integer "collisionGroup"
---@param collisionMask integer "collisionMask"
function moveCCT(characterIndex, x, y, z, collisionGroup, collisionMask) end

--- Overlap box rigid body objects synchronously/blocking
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted, and no more work is done
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param rx number "rx"
---@param ry number "ry"
---@param rz number "rz"
---@param ex number "ex"
---@param ey number "ey"
---@param ez number "ez"
---@param callbackFunctionName string "overlapBoxCallback(nodeId, subShapeIndex) -> boolean continueChecking"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false]"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function overlapBox(x, y, z, rx, ry, rz, ex, ey, ez, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap box rigid body objects asynchronously
--- If the callback function returns true any possible following callbacks will be omitted
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param rx number "rx"
---@param ry number "ry"
---@param rz number "rz"
---@param ex number "ex"
---@param ey number "ey"
---@param ez number "ez"
---@param callbackFunctionName string "overlapBoxAsyncCallback(nodeId, subShapeIndex, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false]"
function overlapBoxAsync(x, y, z, rx, ry, rz, ex, ey, ez, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap rigid body objects with an arbitrary convex mesh synchronously/blocking
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted, and no more work is done
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param shapeId entityId "convex shape to use for overlap, has to have 'CPU Mesh' flag set"
---@param callbackFunctionName string "overlapConvexCallback(nodeId, subShapeIndex) -> boolean continueChecking"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false] (Non-exact is not implemented yet and it will always use exact test)"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function overlapConvex(shapeId, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap rigid body objects with an arbitrary convex mesh asynchronously
--- If the callback function returns true any possible following callbacks will be omitted
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param shapeId entityId "convex shape to use for overlap, has to have 'CPU Mesh' flag set"
---@param callbackFunctionName string "overlapConvexAsyncCallback(nodeId, subShapeIndex, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false] (Non-exact is not implemented yet and it will always use exact test)"
function overlapConvexAsync(shapeId, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap rigid body objects with a convex polyhedron shape synchronously/blocking
--- Convexity is not validated
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted, and no more work is done
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param shape floatArray "convex polyhedron shape to use for overlap. List of points in the format {x0,y0,z0,...,xn,yn,zn}"
---@param callbackFunctionName string "overlapConvexCallback(nodeId, subShapeIndex) -> boolean continueChecking"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false] (Non-exact is not implemented yet and it will always use exact test)"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function overlapConvexPolyhedron(shape, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap rigid body objects with a convex polyhedron shape asynchronously
--- Convexity is not validated
--- If the callback function returns true any possible following callbacks will be omitted
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param shape floatArray "convex polyhedron shape to use for overlap. List of points in the format {x0,y0,z0,...,xn,yn,zn}"
---@param callbackFunctionName string "overlapConvexAsyncCallback(nodeId, subShapeIndex, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false] (Non-exact is not implemented yet and it will always use exact test)"
function overlapConvexPolyhedronAsync(shape, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap cylinder rigid body objects synchronously/blocking
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted, and no more work is done
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param radius number "radius"
---@param height number "height"
---@param axis integer "axis"
---@param callbackFunctionName string "overlapCylinderCallback(nodeId, subShapeIndex) -> boolean continueChecking"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false]"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function overlapCylinder(x, y, z, radius, height, axis, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap cylinder rigid body objects asynchronously
--- If the callback function returns true any possible following callbacks will be omitted
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param radius number "radius"
---@param height number "height"
---@param axis integer "axis"
---@param callbackFunctionName string "overlapCylinderAsyncCallback(nodeId, subShapeIndex, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false]"
function overlapCylinderAsync(x, y, z, radius, height, axis, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap sphere rigid body objects synchronously/blocking
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted, and no more work is done
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param radius number "radius"
---@param callbackFunctionName string "overlapSphereCallback(nodeId, subShapeIndex) -> boolean continueChecking"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false]"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function overlapSphere(x, y, z, radius, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Overlap sphere rigid body objects asynchronously
--- If the callback function returns true any possible following callbacks will be omitted
--- Note that a "dynamic" object is considered to be any object which is not static or kinematic
--- Note that supplying false values to all of includeDynamics, includeKinematics and includeStatics will set them all to true
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param radius number "radius"
---@param callbackFunctionName string "overlapSphereAsyncCallback(nodeId, subShapeIndex, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@param includeDynamics boolean|nil "includeDynamics [optional, default=true]"
---@param includeKinematics boolean|nil "includeKinematics [optional, default=true]"
---@param includeStatics boolean|nil "includeStatics  [optional, default=true]"
---@param exactTest boolean|nil "exactTest [optional, default=false]"
function overlapSphereAsync(x, y, z, radius, callbackFunctionName, callbackTargetObject, collisionMask, includeDynamics, includeKinematics, includeStatics, exactTest) end

--- Pause streamed sample object
---@param streamedSampleId entityId "streamedSampleId"
function pauseStreamedSample(streamedSampleId) end

--- Pin a shared I3D file into the cache, so it's not auto-deleted
---@param filename string "filename"
---@param verbose boolean "verbose"
function pinSharedI3DFileInCache(filename, verbose) end

--- Play sample object
---@param sampleId entityId "sampleId"
---@param loops integer "loops"
---@param volume number "volume"
---@param offset number "offset to start playing [ms]"
---@param delay number "delay until to start playing [ms]"
---@param playAfterSample entityId "optionally wait until playAfterSample has finished playing"
function playSample(sampleId, loops, volume, offset, delay, playAfterSample) end

--- Play streamed sample object
---@param streamedSampleId entityId "streamedSampleId"
---@param repeatCount integer "repeat count"
function playStreamedSample(streamedSampleId, repeatCount) end

--- Print given values to console
--- Each argument is printed in a separate line
--- Can handle all data types without explicit casting
---@param arg1 any|nil "arg1 [optional]"
---@param arg2 any|nil "arg2 [optional]"
---@param arg3 any|nil "arg3 [optional]"
---@param arg4 any|nil "arg4 [optional]"
---@param arg5 any|nil "arg5 [optional]"
---@param arg6 any|nil "arg6 [optional]"
---@param arg7 any|nil "arg7 [optional]"
---@param arg8 any|nil "arg8 [optional]"
function print(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) end

--- print callstack

function printCallstack() end

--- Print given values to console
--- Each argument is printed in a separate line
--- Can handle all data types without explicit casting
--- Text is colored orange in the script console and triggers automatic opening of console if "openOnWarning" setting is enabled
---@param arg1 any|nil "arg1 [optional]"
---@param arg2 any|nil "arg2 [optional]"
---@param arg3 any|nil "arg3 [optional]"
---@param arg4 any|nil "arg4 [optional]"
---@param arg5 any|nil "arg5 [optional]"
---@param arg6 any|nil "arg6 [optional]"
---@param arg7 any|nil "arg7 [optional]"
---@param arg8 any|nil "arg8 [optional]"
function printError(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) end

--- Print given values to console
--- Each argument is printed in a separate line
--- Can handle all data types without explicit casting
--- Text is colored red in the script console and triggers automatic opening of console if "openOnError" setting is enabled
---@param arg1 any|nil "arg1 [optional]"
---@param arg2 any|nil "arg2 [optional]"
---@param arg3 any|nil "arg3 [optional]"
---@param arg4 any|nil "arg4 [optional]"
---@param arg5 any|nil "arg5 [optional]"
---@param arg6 any|nil "arg6 [optional]"
---@param arg7 any|nil "arg7 [optional]"
---@param arg8 any|nil "arg8 [optional]"
function printWarning(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) end

--- Transform vector from world space into screen space
---@param wx number "wx"
---@param wy number "wy"
---@param wz number "wz"
---@return sx float "sx"
function project(wx, wy, wz) end

--- Project world position to screen for specified camera
---@param cameraId entityId "cameraId"
---@param aspectRatio number
---@param x number
---@param y number
---@param z number
---@return x float "screenspace x coordinate"
function projectToCamera(cameraId, aspectRatio, x, y, z) end

--- Raycast rigid body objects synchronously
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param nx number "nx"
---@param ny number "ny"
---@param nz number "nz"
---@param maxDistance number "maxDistance"
---@param callbackFunctionName string "raycastAllCallback(actorId, x, y, z, distance, nx, ny, nz, subShapeIndex, shapeId, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional]"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function raycastAll(x, y, z, nx, ny, nz, maxDistance, callbackFunctionName, callbackTargetObject, collisionMask) end

--- Raycast rigid body objects asynchronously
--- If the callback function returns true any possible following callbacks will be omitted
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param nx number "nx"
---@param ny number "ny"
---@param nz number "nz"
---@param maxDistance number "maxDistance"
---@param callbackFunctionName string "raycastAllAsyncCallback (actorId, x, y, z, distance, nx, ny, nz, subShapeIndex, shapeId, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional]"
function raycastAllAsync(x, y, z, nx, ny, nz, maxDistance, callbackFunctionName, callbackTargetObject, collisionMask) end

--- Raycast closest rigid body object synchronously/blocking
--- Callbacks will be performed before the function returns
--- If the callback function returns true any possible following callbacks will be omitted
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param nx number "nx"
---@param ny number "ny"
---@param nz number "nz"
---@param maxDistance number "maxDistance"
---@param callbackFunctionName string "raycastClosestCallback (nodeId, x,y,z, distance, nx,ny,nz, subShapeIndex, shapeId, isLast) -> boolean continueReporting "
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
---@return numShapes integer "number of hit shapes, equal to number of performed callbacks"
function raycastClosest(x, y, z, nx, ny, nz, maxDistance, callbackFunctionName, callbackTargetObject, collisionMask) end

--- Raycast closest rigid body object asynchronously
--- If the callback function returns true any possible following callbacks will be omitted
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param nx number "nx"
---@param ny number "ny"
---@param nz number "nz"
---@param maxDistance number "maxDistance"
---@param callbackFunctionName string "raycastClosestAsync(actorId, x,y,z, distance, nx,ny,nz, subShapeIndex, shapeId, isLast) -> boolean continueReporting"
---@param callbackTargetObject object|nil "targetObject [optional]"
---@param collisionMask integer|nil "collisionMask [optional, default=ALL_BITS]"
function raycastClosestAsync(x, y, z, nx, ny, nz, maxDistance, callbackFunctionName, callbackTargetObject, collisionMask) end

--- Read fill plane surface information from stream
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param streamId entityId "streamId"
---@param totalVolume number "total volume of the fill plane"
---@return success boolean
function readFillPlaneFromStream(fillPlaneShapeId, streamId, totalVolume) end

--- Reduces the ref count of the given shared i3d. Must be called for every successfull loadSharedI3DFile and streamSharedI3DFile call to avoid memory leaks.
---@param requestId integer "stream I3D request ID"
---@param warnIfInvalid boolean|nil "[optional] print a warning if the request ID is invalid, defaults to false"
function releaseSharedI3DFile(requestId, warnIfInvalid) end

--- remove all differential
---@param transformId entityId "transformId"
function removeAllDifferentials(transformId) end

--- Remove a sample element from an AudioSource
---@param audioSource entityId "audioSource"
---@param sampleElementIndex integer "sampleElementIndex"
function removeAudioSourceSampleElement(audioSource, sampleElementIndex) end

--- Remove character controller
---@param characterIndex integer "characterIndex"
function removeCCT(characterIndex) end

--- Remove from physics
---@param transformId entityId "transformId"
function removeFromPhysics(transformId) end

--- (Recursively) removes given entity or its shape children from the ignore list for merged shadow rendering
---@param lightId entityId "id of the light source node"
---@param id shapeId "of shape or parent node of a set of shapes (such as a vehicle)."
function removeMergedShadowIgnoreShapes(lightId, id) end

--- Remove spline attribute
---@param shapeId integer
---@param attributeIndex integer "attribute index (0 based)"
function removeSplineAttribute(shapeId, attributeIndex) end

--- Remove user attribute by name
---@param objectId entityId "objectId"
---@param attributeName string "name of the attribute to remove"
function removeUserAttribute(objectId, attributeName) end

--- Remove XML property.
---@param xmlId entityId "xmlId"
---@param xmlPath string "Path to element or attribute"
---@return success boolean "success"
function removeXMLProperty(xmlId, xmlPath) end

--- Render overlay
--- Only allowed to be called within "draw"
---@param overlayId entityId "overlayId"
---@param x number "screenspace x [0 ..1]"
---@param y number "screenspace y [0 ..1]"
---@param width number "normalized width, 1 = full screen width"
---@param height number "normalized height 1 = full screen height"
function renderOverlay(overlayId, x, y, width, height) end

--- Render text to viewport
--- Screenspace coordinates origin is bottom left corner.
---@param x number "screenspace x [0..1]"
---@param y number "screenspace y [0..1]"
---@param fontSize number "fontSize"
---@param string string "string to render, can contain linebreaks"
function renderText(x, y, fontSize, string) end

--- Resets the start timer of emitted particles.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param time float|nil "scale time scale [optional]"
function resetEmitStartTimer(particleSystemId, time) end

--- Resets the stop timer of emitted particles.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param time float|nil "scale time scale [optional]"
function resetEmitStopTimer(particleSystemId, time) end

--- Resets the counter of emitted particles. This is used if the maxEmit attribute is set for the particle system.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
function resetNumOfEmittedParticles(particleSystemId) end

--- Resume streamed sample
---@param streamedSampleId entityId "streamedSampleId"
function resumeStreamedSample(streamedSampleId) end

--- Rotate transform about given local axis
---@param transformId entityId "transformId"
---@param rotation number "rotation in radians"
---@param dx number "axis x direction"
---@param dy number "axis y direction"
---@param dz number "axis z direction"
function rotateAboutLocalAxis(transformId, rotation, dx, dy, dz) end

--- Save XML file to preset path
---@param xmlId entityId "xmlId"
---@return success boolean "success"
function saveXMLFile(xmlId) end

--- Get XML as xml string
---@param xmlId entityId "xmlId"
---@return xmlString string
function saveXMLFileToMemory(xmlId) end

--- Set mask for 2d rendering (font and overlays)
---@param textureId entityId
---@param maskAlphaOnly boolean "if true, applies the mask to the alpha channel only, otherwise to all 4 channels"
---@param x number "x position"
---@param y number "y position"
---@param width number
---@param height number
function set2DMaskFromOverlay(textureId, maskAlphaOnly, x, y, width, height) end

--- Set mask for 2d rendering (font and overlays)
---@param textureId entityId
---@param maskAlphaOnly boolean "if true, applies the mask to the alpha channel only, otherwise to all 4 channels"
---@param x number "x position"
---@param y number "y position"
---@param width number
---@param height number
function set2DMaskFromTexture(textureId, maskAlphaOnly, x, y, width, height) end

--- Set angular damping
---@param transformId entityId "transformId"
---@param angularDamping number "angularDamping"
function setAngularDamping(transformId, angularDamping) end

--- Set angular velocity of transform object
---@param transformId entityId "transformId"
---@param velocityX number "velocityX"
---@param velocityY number "velocityY"
---@param velocityZ number "velocityZ"
function setAngularVelocity(transformId, velocityX, velocityY, velocityZ) end

--- Set animation track blend weight
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@param weight number "weight"
function setAnimTrackBlendWeight(characterSetId, trackId, weight) end

--- Set animation track loop state
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@param loopState boolean "loopState"
function setAnimTrackLoopState(characterSetId, trackId, loopState) end

--- Set animation track speed scale
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@param speedScale number "speedScale"
function setAnimTrackSpeedScale(characterSetId, trackId, speedScale) end

--- Set animation track time
---@param characterSetId entityId "characterSetId"
---@param trackId integer "trackId"
---@param time number "time"
---@param immediateUpdate boolean|nil "immediateUpdate [optional]"
function setAnimTrackTime(characterSetId, trackId, time, immediateUpdate) end

--- Set audio culling world properties
---@param gridMinX number "gridMinX"
---@param gridMinY number "gridMinY"
---@param gridMinZ number "gridMinZ"
---@param gridMaxX number "gridMaxX"
---@param gridMaxY number "gridMaxY"
---@param gridMaxZ number "gridMaxZ"
---@param gridSize integer "gridSize"
---@param clipDistanceThreshold1 number "clipDistanceThreshold1"
---@param clipDistanceThreshold2 number "clipDistanceThreshold2"
function setAudioCullingWorldProperties(gridMinX, gridMinY, gridMinZ, gridMaxX, gridMaxY, gridMaxZ, gridSize, clipDistanceThreshold1, clipDistanceThreshold2) end

--- Set audio source auto play enabled
---@param audioSource entityId "audioSource"
---@param autoPlayEnabled boolean
function setAudioSourceAutoPlay(audioSource, autoPlayEnabled) end

--- Sets inner range of audio source. If the camera is further than this distance volume will start to decrease
---@param audioSource entityId "audioSource"
---@param range number "inner range (fade start)"
function setAudioSourceInnerRange(audioSource, range) end

--- Set the AudioSource's max retrigger delay
---@param audioSource entityId "audioSource"
---@param maxRetriggerDelay number "maxRetriggerDelay"
function setAudioSourceMaxRetriggerDelay(audioSource, maxRetriggerDelay) end

--- Get the AudioSource's min retrigger delay
---@param audioSource entityId "audioSource"
---@param minRetriggerDelay number "minRetriggerDelay"
function setAudioSourceMinRetriggerDelay(audioSource, minRetriggerDelay) end

--- Set audio source priority.
---@param audioSource entityId "audioSource"
---@param priority integer
function setAudioSourcePriority(audioSource, priority) end

--- Set the AudioSource's random playback state
---@param audioSource entityId "audioSource"
---@param randomPlayback boolean "randomPlayback"
function setAudioSourceRandomPlayback(audioSource, randomPlayback) end

--- Sets (outer) range of audio source
---@param audioSource entityId "audioSource"
---@param range number "outer range (fade end)"
function setAudioSourceRange(audioSource, range) end

--- Set the AudioSource's sample element's probability
---@param audioSource entityId "audioSource"
---@param index integer "index"
---@param probability number "probability"
function setAudioSourceSampleElementProbability(audioSource, index, probability) end

--- Set the inaudible behavior of the sound. By default, if a sound is inaudible, it's paused, and will resume when it becomes audible again.
---@param audioSource entityId "audioSource"
---@param tickIfInaudible boolean
function setAudioSourceTickInaudible(audioSource, tickIfInaudible) end

--- Set active camera
---@param cameraId entityId "cameraId"
function setCamera(cameraId) end

--- Set character height (height will be reverted if the resizing leads to a collision)
---@param characterIndex integer "characterIndex"
---@param height number "height"
---@param collisionGroup integer "collisionGroup"
---@param collisionMask integer "collisionMask"
function setCCTHeight(characterIndex, height, collisionGroup, collisionMask) end

--- Set center of mass (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function setCenterOfMass(transformId, x, y, z) end

--- Set object clip distance
---@param objectId entityId "objectId"
---@param distance number "distance"
function setClipDistance(objectId, distance) end

--- Set collision filter
---@param transformId entityId "transformId"
---@param group integer "group"
---@param mask integer "mask"
function setCollisionFilter(transformId, group, mask) end

--- Set collision filter group (mask remains the same)
---@param transformId entityId "transformId"
---@param group integer "group"
function setCollisionFilterGroup(transformId, group) end

--- Set collision filter mask (group remains the same)
---@param transformId entityId "transformId"
---@param mask integer "mask"
function setCollisionFilterMask(transformId, mask) end

--- Set boolean value
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param parameterId integer "parameterId"
---@param value boolean "value"
---@return success boolean "success"
function setConditionalAnimationBoolValue(conditionalAnimationEntityId, parameterId, value) end

--- Set float value
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param parameterId integer "parameterId"
---@param value number "value"
---@return success boolean "success"
function setConditionalAnimationFloatValue(conditionalAnimationEntityId, parameterId, value) end

--- Set specific parameter ids
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param velocityParmId integer "velocityParmId"
---@param angularVelocityParmId integer "angularVelocityParmId"
function setConditionalAnimationSpecificParameterIds(conditionalAnimationEntityId, velocityParmId, angularVelocityParmId) end

--- Set time
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param time number
function setConditionalAnimationTime(conditionalAnimationEntityId, time) end

--- 
---@param densityMapSyncerId entityId
---@param densityMapId entityId
---@param callbackFunctionName string "callback function(densityMapId, cellX, cellZ)"
---@param target object|nil "target [optional]"
function setDensityMapSyncerCellChangedCallback(densityMapSyncerId, densityMapId, callbackFunctionName, target) end

--- Set the direction of an object, the positive z-axis points towards the given direction. The y-axis lies in the direction-up-plane.
---@param transformId entityId "transformId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param upX number "upX"
---@param upY number "upY"
---@param upZ number "upZ"
function setDirection(transformId, x, y, z, upX, upY, upZ) end

--- Set particle system count scale
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param countScale number "countScale"
function setEmitCountScale(particleSystemId, countScale) end

--- Set emitter starting time.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param emitStartTime number "emitStartTime"
function setEmitStartTime(particleSystemId, emitStartTime) end

--- Set emitter stop time.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param emitStopTime number "emitStopTime"
function setEmitStopTime(particleSystemId, emitStopTime) end

--- Sets the emitter shape of the particle system
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param shapeId entityId "shapeId"
function setEmitterShape(particleSystemId, shapeId) end

--- Sets the emitter shape velocity scale of the particle system
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param shape number "velocity scale"
function setEmitterShapeVelocityScale(particleSystemId, shape) end

--- Set whether the particle system should emit new particles
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param state boolean "state"
function setEmittingState(particleSystemId, state) end

--- 
---@param densityMapSyncer entityId
---@param densityMapId entityId
---@param cellX integer "cell index"
---@param cellZ integer "cell index"
---@param enable boolean|nil "enable flag (defaults to true) [optional]"
function setEnableDensityMapSyncerCellChangedCallback(densityMapSyncer, densityMapId, cellX, cellZ, enable) end

--- Set camera far clip distance
---@param cameraId entityId "cameraId"
---@param farClip number "farClip"
function setFarClip(cameraId, farClip) end

--- Set fast shadow update for camera
---@param cameraId entityId "cameraId"
---@param fastUpdate boolean "set true for fast shadow update or false for far shadows"
function setFastShadowUpdate(cameraId, fastUpdate) end

--- Set fill plane physical surface angle
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param physicalSurfAngle number "physicalSurfAngle"
function setFillPlaneMaxPhysicalSurfaceAngle(fillPlaneShapeId, physicalSurfAngle) end

--- 
---@param foliageBendingSystemId entityId
---@param rectangleId integer
---@param minX number
---@param maxX number
---@param minZ number
---@param maxZ number
---@param yOffset number
function setFoliageBendingRectangleAttributes(foliageBendingSystemId, rectangleId, minX, maxX, minZ, maxZ, yOffset) end

--- 
---@param terrainId entityId "terrainId"
---@param foliageBendingSystemId entityId "foliageBendingSystemId"
function setFoliageBendingSystem(terrainId, foliageBendingSystemId) end

--- Sets the vertical field of view angle
---@param cameraId entityId "id of the camera"
---@param fovY number "field of view angle (radian)"
function setFovY(cameraId, fovY) end

--- Sets friction velocity to collision
---@param objectId entityId "objectId"
---@param velocity number "velocity"
function setFrictionVelocity(objectId, velocity) end

--- Set if transform group has physics collision
---@param transformId entityId
---@param hasCollision boolean
function setHasCollision(transformId, hasCollision) end

--- Set center of mass (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function setInertiaScale(transformId, x, y, z) end

--- Set rigid body transform as compound. Required to not be in physics simulation when called
---@param transformId entityId "transformId"
---@param isCompound boolean "isCompound"
function setIsCompound(transformId, isCompound) end

--- Set rigid body transform as compound child. Required to not be in physics simulation when called
---@param transformId entityId "transformId"
---@param isCompoundChild boolean "isCompoundChild"
function setIsCompoundChild(transformId, isCompoundChild) end

--- Set if shape is non-renderable
---@param shapeId entityId "shapeId"
---@param isNonRenderable boolean
function setIsNonRenderable(shapeId, isNonRenderable) end

--- Set camera orthographic mode
---@param cameraId entityId "cameraId"
---@param isOrthographic boolean
function setIsOrthographic(cameraId, isOrthographic) end

--- Set joint angular drive
---@param jointIndex integer
---@param axis integer
---@param usePosition boolean
---@param useVelocity boolean
---@param spring number
---@param damping number
---@param forceLimit number
---@param targetRotation number
---@param targetAngularVelocity number
function setJointAngularDrive(jointIndex, axis, usePosition, useVelocity, spring, damping, forceLimit, targetRotation, targetAngularVelocity) end

--- Set joint drive. Drives orientation if position drive or angular velocity if velocity drive.
---@param jointIndex integer
---@param isLinear boolean
---@param isPosition boolean
---@param valueX number
---@param valueY number
---@param valueZ number
---@param angle number
function setJointDrive(jointIndex, isLinear, isPosition, valueX, valueY, valueZ, angle) end

--- Set joint linear drive
---@param jointIndex integer
---@param axis integer "[0-2]"
---@param usePosition boolean
---@param useVelocity boolean
---@param spring number
---@param damping number
---@param forceLimit number
---@param targetPosition number
---@param targetVelocity number
function setJointLinearDrive(jointIndex, axis, usePosition, useVelocity, spring, damping, forceLimit, targetPosition, targetVelocity) end

--- Set light cone angle in radian
---@param lightId entityId
---@param cone number "angle"
function setLightConeAngle(lightId, cone) end

--- Set light dropoff
---@param lightId entityId
---@param dropoff number
function setLightDropOff(lightId, dropoff) end

--- Set IES Light profile as a path to a *.ies file.
---@param lightId entityId
---@param ___ies_ string "filepath "
function setLightIESProfile(lightId, ___ies_) end

--- Set range of a light
---@param lightId entityId "lightId"
---@param range number "range"
function setLightRange(lightId, range) end

--- 
---@param lightId entityId
---@param dirX number
---@param dirY number
---@param dirZ number
function setLightScatteringColor(lightId, dirX, dirY, dirZ) end

--- 
---@param lightId entityId
---@param scattering number "cone angle"
function setLightScatteringConeAngle(lightId, scattering) end

--- 
---@param lightId entityId
---@param dirX number
---@param dirY number
---@param dirZ number
function setLightScatteringDirection(lightId, dirX, dirY, dirZ) end

--- 
---@param lightId entityId
---@param scattering number "intensity"
function setLightScatteringIntensity(lightId, scattering) end

--- Set light shadow map
---@param lightId entityId "lightId"
---@param castShowMap boolean
---@param depthMapResolution integer
function setLightShadowMap(lightId, castShowMap, depthMapResolution) end

--- Sets shadow priority (float value) for the given shadow light. Higher value means higher priority (will be picked before lower priority lights when too many shadows are on the screen).
---@param lightId entityId "id of the light source"
---@param shadow shadowPriority "priority value of the light source"
function setLightShadowPriority(lightId, shadow) end

--- Sets soft shadow depth bias factor of light source. The bias factor is multiplied with the depth bias of the light source (e.g. depth bias = 0.0001f, bias factor = 2.0f -> depth bias is 0.0002f). They are separated so you can still have the normal bias for PCF shadows (when soft shadows are disabled), which generally
--- need a smaller bias.
---@param lightId entityId "id of the light source"
---@param depth softShadowDistance "bias factor for soft shadows used by the light source"
function setLightSoftShadowDepthBiasFactor(lightId, depth) end

--- Sets soft shadow light distance for directional lights (it's fake, fixed distance from each pixel). Ignored by spot lights.
---@param lightId entityId "id of the light source"
---@param Distance softShadowDistance "(in meters) of the fake area light source to the ground"
function setLightSoftShadowDistance(lightId, Distance) end

--- Sets soft shadow size. This is essentially the size of the virtual/imagined light source that is casting the soft shadow (for directional lights, instead of an infinitely far away sun, it's a fake square light source). The size of the shadow on the floor is then a function of this size, the light source distance from the ground,
--- and the distance of shadow blocker to the ground.
---@param lightId entityId "id of the light source"
---@param soft softShadowSize "shadow light size"
function setLightSoftShadowSize(lightId, soft) end

--- 
---@param lightId entityId
---@param useLightScattering boolean
function setLightUseLightScattering(lightId, useLightScattering) end

--- Set linear damping
---@param transformId entityId "transformId"
---@param linearDamping number "linearDamping"
function setLinearDamping(transformId, linearDamping) end

--- Set linear velocity of transform object
---@param transformId entityId "transformId"
---@param velocityX number "velocityX"
---@param velocityY number "velocityY"
---@param velocityZ number "velocityZ"
function setLinearVelocity(transformId, velocityX, velocityY, velocityZ) end

--- Set transform object locked group flag
---@param transformId entityId "transformId"
---@param locked boolean "group locked group"
function setLockedGroup(transformId, locked) end

--- Set mass
---@param transformId entityId "transformId"
---@param mass number "mass in tons"
function setMass(transformId, mass) end

--- Set shape material at index
---@param shapeId entityId "shapeId"
---@param material entityId "materialId of the material entity"
---@param materialIndex integer "material index of shape to set, starting at 0"
function setMaterial(shapeId, material, materialIndex) end

--- 
---@param materialId entityId
---@param name string
---@param filename string
---@param textueWrap boolean|nil "[optional, default defined by custom shader; ignored anyway]"
---@param isSRGB boolean|nil "[optional, default defined by custom shader]"
---@param sharedEdit boolean|nil "[optional, default=true]"
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialCustomMapFromFile(materialId, name, filename, textueWrap, isSRGB, sharedEdit) end

--- Set the custom parameter values of a material
---@param materialId entityId
---@param name string
---@param x number
---@param y number
---@param z number
---@param w number
---@param sharedEdit boolean|nil "[optional, default=true]"
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialCustomParameter(materialId, name, x, y, z, w, sharedEdit) end

--- 
---@param materialId entityId
---@param shaderVariationName string
---@param sharedEdit boolean
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialCustomShaderVariation(materialId, shaderVariationName, sharedEdit) end

--- 
---@param materialId entityId
---@param filename string
---@param textueWrap boolean "(ignored)"
---@param isSRGB boolean
---@param sharedEdit boolean
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialDiffuseMapFromFile(materialId, filename, textueWrap, isSRGB, sharedEdit) end

--- 
---@param materialId entityId
---@param name string
---@param size size
---@param sharedEdit boolean|nil "[optional, default=true]"
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialDynamicArrayOfTextureSize(materialId, name, size, sharedEdit) end

--- 
---@param materialId entityId
---@param filename string
---@param textueWrap boolean "(ignored)"
---@param isSRGB boolean
---@param sharedEdit boolean
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialGlossMapFromFile(materialId, filename, textueWrap, isSRGB, sharedEdit) end

--- 
---@param materialId entityId
---@param filename string
---@param textueWrap boolean "(ignored)"
---@param isSRGB boolean
---@param sharedEdit boolean
---@return newMaterialId entityId "material id of new material (same as materialId with shared edit mode)"
function setMaterialNormalMapFromFile(materialId, filename, textueWrap, isSRGB, sharedEdit) end

--- Sets the shapes slot name for the material at the given index.
---@param shapeId entityId "shapeId"
---@param materialIndex integer "material index of shape, starting at 0"
---@param slotName string "slotName for the material at the given index or an empty string to remove the slot name"
function setMaterialSlotName(shapeId, materialIndex, slotName) end

--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param maxNumParticles integer "maximum number of active particles"
function setMaxNumOfParticles(particleSystemId, maxNumParticles) end

--- For the given light source, enables or disables the entire merged shadow (i.e. for all lights in the group). Can be used to toggle shadows e.g. for comparison purposes.
---@param lightId entityId "id of the light source node"
---@param shouldEnable boolean "enables (true) or disables (false) the merged shadow group"
function setMergedShadowActive(lightId, shouldEnable) end

--- Sets the soft shadow light size for the merged shadow of the given light source (if it belongs to any merged shadow).
---@param lightId entityId "id of the light source node"
---@param lightSize number "soft shadow light size of the merged shadow"
function setMergedShadowLightSize(lightId, lightSize) end

--- Sets the near plane for culling and rendering of the given merged shadow. Useful if the center of the merged shadow ends up inside objects and you want it to only start rendering at some distance from the center.
---@param lightId entityId "id of the light source node"
---@param nearPlane number "near plane of the merged shadow"
function setMergedShadowNearPlaneOffset(lightId, nearPlane) end

--- Sets various shadow settings of the merged shadow that this light belongs to (sets it for all lights in the group). Arguments can be nil if you'd like one of them to remain unchanged.
--- By default, these are simply set to the equivalent settings of the very first light that was added to the merged shadow group.
--- If you are unsure about the exact values of some settings, you can hand in the equivalent setting of one of the lights in the group (e.g. depth bias, shadow map resolution, etc.).
---@param lightId entityId "id of the light source node"
---@param depthBias float|nil "shadow map depth bias for the merged shadow [optional, default=currentValue]"
---@param slopeScaleDepthBias float|nil "slope-scaled shadow map depth bias for the merged shadow [optional, default=currentValue]"
---@param slopeScaleDepthClamp float|nil "clamp for the slope-sclaed shadow map depth bias for the merged shadow [optional, default=currentValue]"
---@param shadowMapResolution integer|nil "resolution of the merged shadow's shadow/depth map, will be used for both width and height [optional, default=currentValue]"
function setMergedShadowSettings(lightId, depthBias, slopeScaleDepthBias, slopeScaleDepthClamp, shadowMapResolution) end

--- Sets the shadow settings light for a group of light sources with merged shadows. This will replace the automatic computation and explicitly use all the shadow settings from the given shadow settings light when rendering the merged shadow.
---@param lightId entityId "id of one of the light sources in a given merged shadow group"
---@param id shadowSettingsLightId "the light source that should be used as a source of settings"
function setMergedShadowSettingsLight(lightId, id) end

--- Set minimum clip distance
---@param objectId entityId "objectId"
---@param minDist number "minDist"
function setMinClipDistance(objectId, minDist) end

--- Set entity name
---@param entityId entityId "entityId"
---@param entityName string "entityName"
function setName(entityId, entityName) end

--- Set camera near clip distance
---@param cameraId entityId "cameraId"
---@param nearClip number "nearClip"
function setNearClip(cameraId, nearClip) end

--- Set note node color
---@param noteId entityId "Id of the note node"
---@param colorR number "R component of note color [0-1]"
---@param colorG number "G component of note color [0-1]"
---@param colorB number "B component of note color [0-1]"
function setNoteNodeColor(noteId, colorR, colorG, colorB) end

--- Set note node fixed size
---@param noteId entityId "Id of the note node"
---@param fixedSize boolean
function setNoteNodeFixedSize(noteId, fixedSize) end

--- Set note node text
---@param noteId entityId "Id of the note node"
---@param text string
function setNoteNodeText(noteId, text) end

--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param emittedParticlesPerMs number "emittedParticlesPerMs"
function setNumOfParticlesToEmitPerMs(particleSystemId, emittedParticlesPerMs) end

--- Set object mask
---@param objectId entityId "objectId"
---@param mask integer "mask"
function setObjectMask(objectId, mask) end

--- Set camera orthographic height
---@param cameraId entityId "cameraId"
---@param orthographicHeight number
function setOrthographicHeight(cameraId, orthographicHeight) end

--- Set overlay color and alpha for whole overlay
---@param overlayId entityId "overlayId"
---@param red number "red"
---@param green number "green"
---@param blue number "blue"
---@param alpha number "alpha"
function setOverlayColor(overlayId, red, green, blue, alpha) end

--- Set overlay color and alpha for specified corner
--- Values are linearly interpolated between the corners allowing for gradients
---@param overlayId entityId "overlayId"
---@param cornerIndex integer "corner index (same order as uvs: 0 = bottom left, 1 = top left, 2 = bottom right, 3 = top right) "
---@param red number "red"
---@param green number "green"
---@param blue number "blue"
---@param alpha number "alpha"
function setOverlayCornerColor(overlayId, cornerIndex, red, green, blue, alpha) end

--- Set layer for texture array overlays
---@param overlayId entityId "overlayId"
---@param layer integer
function setOverlayLayer(overlayId, layer) end

--- Set overlay rotation
---@param overlayId entityId "overlayId"
---@param rotation number "rotation"
---@param pivotX number "x position of pivot of rotation (relative to position of overlay)"
---@param pivotY number "y position of pivot of rotation (relative to position of overlay)"
function setOverlayRotation(overlayId, rotation, pivotX, pivotY) end

--- Set overlay signed distance field width
---@param overlayId entityId "overlayId"
---@param sdfWidth number "range of the sdf in normalized color (0.5 full range, 0 disabled)"
function setOverlaySignedDistanceFieldWidth(overlayId, sdfWidth) end

--- Set overlay uv coordinates
---@param overlayId entityId "overlayId"
---@param u0 number "bottom left u tex coord (original = 0)"
---@param v0 number "bottom left v tex coord (original = 0)"
---@param u1 number "top left u tex coord (original = 0)"
---@param v1 number "top left v tex coord (original = 1)"
---@param u2 number "bottom right u tex coord (original = 1)"
---@param v2 number "bottom right v tex coord (original = 0)"
---@param u3 number "top right u tex coord (original = 1)"
---@param v3 number "top right v tex coord (original = 1)"
function setOverlayUVs(overlayId, u0, v0, u1, v1, u2, v2, u3, v3) end

--- Sets the particle shape of the particle system
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param nodeId entityId "nodeId"
function setParticleShape(particleSystemId, nodeId) end

--- Set particle system life span.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param lifeSpan number "lifeSpan"
---@param keepBlendTimes boolean "keepBlendTimes"
function setParticleSystemLifespan(particleSystemId, lifeSpan, keepBlendTimes) end

--- Set particle system speed
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param speed number "speed"
function setParticleSystemSpeed(particleSystemId, speed) end

--- Sets the time scale for the particle simulation.
--- The particleSystemId can be retrieved by using getGeometry() on the shape/node
---@param particleSystemId entityId "particleSystemId"
---@param timeScale number "timeScale"
function setParticleSystemTimeScale(particleSystemId, timeScale) end

--- Translate polyline
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param polyLineIdx integer "polyLineIdx"
---@param dx number "dx"
---@param dz number "dz"
function setPolylineTranslation(fillPlaneShapeId, polyLineIdx, dx, dz) end

--- Set camera projection offset
---@param cameraId entityId "cameraId"
---@param x number "x"
---@param y number "y"
function setProjectionOffset(cameraId, x, y) end

--- Set quaternion in local space
---@param objectId entityId "objectId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param w number "w"
function setQuaternion(objectId, x, y, z, w) end

--- Set shape reflection map scaling
---@param shapeId entityId "shapeId"
---@param scale number "scale"
---@param shared boolean
function setReflectionMapScaling(shapeId, scale, shared) end

--- Set rigid body type
---@param transformId entityId "transformId"
---@param type integer "one of enum RIGID_BODY_TYPE"
function setRigidBodyType(transformId, type) end

--- Set rootnode
---@param transformId entityId "transformId"
function setRootNode(transformId) end

--- Set local space rotation of a transform (relative to its parent)
---@param objectId entityId "objectId"
---@param x number "x rotation in radians"
---@param y number "y rotation in radians"
---@param z number "z rotation in radians"
function setRotation(objectId, x, y, z) end

--- Set sample pitch
---@param sampleId entityId "sampleId"
---@param pitch number "pitch"
function setSamplePitch(sampleId, pitch) end

--- Set velocity of a sample object
---@param sampleId entityId "sampleId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function setSampleVelocity(sampleId, x, y, z) end

--- Set sample volume
---@param sampleId entityId "sampleId"
---@param volume number "volume"
function setSampleVolume(sampleId, volume) end

--- Set scale of a transform object
---@param transformId entityId "transformId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
function setScale(transformId, x, y, z) end

--- Set shader parameter
---@param shapeId entityId "shapeId"
---@param parameterName string "parameterName"
---@param x float|nil "[optional, default=currentValue]"
---@param y float|nil "[optional, default=currentValue]"
---@param z float|nil "[optional, default=currentValue]"
---@param w float|nil "[optional, default=currentValue]"
---@param shared boolean|nil "[optional, default=true]"
---@param materialIndex integer|nil "material index or a negative value to set to all materials [optional, default=-1]"
function setShaderParameter(shapeId, parameterName, x, y, z, w, shared, materialIndex) end

--- Set active shadow focus box
---@param shapeId entityId "shapeId to use as shadowFocusBox or 0 to reset to none"
function setShadowFocusBox(shapeId) end

--- Sets a fake height that is added to the terrain height below the water rest level. This makes the water deeper than it really is, which causes waves to move faster and be deeper. For visual tuning purposes.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param height number "fake height to add below the water rest level"
function setShallowWaterSimulationFakeExtraDepth(shallowWaterSimulation, height) end

--- Sets foam acccumulation rate (determines how much and how quickly foam spawns in turbulent waters).
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param foam number "accumulation rate"
function setShallowWaterSimulationFoamAccumulationRate(shallowWaterSimulation, foam) end

--- Sets foam decay rate (how quickly foam decays after it has spawned).
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param foam number "decay rate"
function setShallowWaterSimulationFoamDecayRate(shallowWaterSimulation, foam) end

--- Manually sets ground height texture of the simulation. Only intended for debugging purposes.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param textureId entityId "texture/id to use as ground height. Has to be an UINT texture."
function setShallowWaterSimulationGroundHeightTexture(shallowWaterSimulation, textureId) end

--- Manually sets ground height texture from a terrain transform group. The terrain transform group has to remain alive for the time the Shallow Water Simulation is alive.
--- When the simulation has a terrain-based ground height texture, the simulation fetches the terrain height correctly based on the world position set from shallowWaterSimulationSetWorldPosition
--- as well as the physical grid size set when the simulation was created.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param terrainTransformGroup integer "transform group/id to get the height texture from"
function setShallowWaterSimulationGroundHeightTextureFromTerrain(shallowWaterSimulation, terrainTransformGroup) end

--- Sets various simulation parameters of the shallow water simulation
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param deltaTime number "time step size for each iteration"
---@param externalAcceleration number "external downwards accelerations (mostly gravity)"
---@param velocityDampening number "artifical dampening factor between 0 and 1 (default: 1.0) that slows down velocities over time and causes the water to calm. even high values (e.g. 0.999) will significantly reduce how turbulent the water looks"
function setShallowWaterSimulationParameters(shallowWaterSimulation, deltaTime, externalAcceleration, velocityDampening) end

--- Sets various parameters used for the perfectly matched layers algorithm
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param dampeningFactor number "controls how quickly and how strongly into the dampening region stuff is dampened."
---@param lambdaUpdateFactor number "controls how quickly the dampening field changes over time. smaller values make the simulation more stable but may cause the dampening effect to worsen"
---@param lambdaDecay number "controls how quickly the dampening field decays per simulation iteration (e.g. 0.9 -> dampening field strength reduce to 90% after one iteration)"
---@param borderSize integer "size of the dampening region in grid cells"
---@param isCubicScaling boolean "determines whether the dampening increases cubically from the start of the dampening border (false means it scales quadratically)."
---@param leftBorder boolean "sets whether PML boundary condition is active for the left border of the simulation"
---@param rightBorder boolean "sets whether PML boundary condition is active for the right border of the simulation"
---@param bottomBorder boolean "sets whether PML boundary condition is active for the bottom border of the simulation"
---@param topBorder boolean "sets whether PML boundary condition is active for the top border of the simulation"
function setShallowWaterSimulationPerfectlyMatchedLayerParameters(shallowWaterSimulation, dampeningFactor, lambdaUpdateFactor, lambdaDecay, borderSize, isCubicScaling, leftBorder, rightBorder, bottomBorder, topBorder) end

--- Sets (total) world space size of the simulation grid.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param width number "world space width of the simulation grid"
---@param height number "world space height of the simulation grid"
function setShallowWaterSimulationPhysicalGridSize(shallowWaterSimulation, width, height) end

--- Sets (2D) world position of the simulation (x and z coordinates). If a height map from a terrain transform group was assigned to the simulation, changing the world position
--- also automatically updates the simulations positioning relative to the terrain height map and translates the simulation contents when the simulation moves.
--- This function automatically snaps the given position to multiples of the grids cell size, and returns the actual position used internally.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param x number "x world space coordinate of the simulation"
---@param z number "z world space coordinate of the simulation"
---@return float
function setShallowWaterSimulationWorldPosition(shallowWaterSimulation, x, z) end

--- Set the bones of the shape given the new root node. The hierarchy of the new bones must be mostly the same as for the currently set bones. Additional children after the used bones are allowed. Bones are matched by node indices and not by name.
--- The new root newRootBoneId must match with oldRootBoneId or the currently assigned root (lowest common root node of all bones) if oldRootBoneId is 0.
---@param shapeId entityId "shapeId"
---@param newRootBoneId entityId "The roof the new skeleton"
---@param oldRootBoneId entityId "The root of the currently assigned skeleton. If 0, the lowest common root node of the currently assigned bones is used"
---@param keepBindPoses boolean|nil "If true, the bind poses of the current bones are kept, otherwise the new bones are assumed to be in the bind pose [optional]"
---@return success boolean
function setShapeBones(shapeId, newRootBoneId, oldRootBoneId, keepBindPoses) end

--- Set the same bones as the other shape uses. Both shapes must use exactly the same number of bones
---@param shapeId entityId "shapeId"
---@param sourceShapeId entityId "sourceShapeId"
---@return success boolean
function setShapeBonesFromShape(shapeId, sourceShapeId) end

--- Set the bounding sphere of this shape only
---@param shapeId entityId
---@param localPosX number
---@param localPosY number
---@param localPosZ number
---@param radius number "if < 0, the bounding sphere of the geometry will be used"
function setShapeBoundingSphere(shapeId, localPosX, localPosY, localPosZ, radius) end

--- Set shape build nav mesh mask
---@param shapeId entityId
---@param mask integer
function setShapeBuildNavMeshMask(shapeId, mask) end

--- Set shape cast shadow map 
---@param shapeId entityId "shapeId"
---@param castShadowmap boolean "castShadowmap"
function setShapeCastShadowmap(shapeId, castShadowmap) end

--- Set shape decal layer
---@param shapeId entityId
---@param decalLayer integer
function setShapeDecalLayer(shapeId, decalLayer) end

--- Set the bounding sphere of the shape's geometry.
--- Warning: This does not update the bounding volumes of other shapes using this geometry. Those are only updated if they are invalided, e.g. by moving the shape or calling invalidShapeBoundingVolume
---@param shapeId entityId
---@param localPosX number
---@param localPosY number
---@param localPosZ number
---@param radius number
function setShapeGeometryBoundingSphere(shapeId, localPosX, localPosY, localPosZ, radius) end

--- Set shape receive shadow map
---@param shapeId entityId "shapeId"
---@param receiveShadowmap boolean "receiveShadowmap"
function setShapeReceiveShadowmap(shapeId, receiveShadowmap) end

--- Set solver iteration count (only for dynamic physics objects)
---@param transformId entityId "transformId"
---@param count integer "count"
function setSolverIterationCount(transformId, count) end

--- Set the spline attribute value at given CV
---@param shapeId integer
---@param attributeIndex integer "attribute index (0 based)"
---@param CVIndex integer
---@param value number
function setSplineAttributeAtCV(shapeId, attributeIndex, CVIndex, value) end

--- Set the position of a spline control point.
---@param shapeId integer "Spline id"
---@param index integer "The index of the control point to be positioned, starting at 0"
---@param x number "New spline point position x in spline localspace"
---@param y number "New spline point position y in spline localspace"
---@param z number "New spline point position z in spline localspace"
function setSplineCV(shapeId, index, x, y, z) end

--- Set the position of a spline edit point.
---@param shapeId integer "Spline id"
---@param index integer "The index of the edit point to be positioned, starting at 0"
---@param x number "New spline point position x in spline localspace"
---@param y number "New spline point position y in spline localspace"
---@param z number "New spline point position z in spline localspace"
function setSplineEP(shapeId, index, x, y, z) end

--- Set streamed sample group
---@param streamedSampleId entityId "streamedSampleId"
---@param group integer "group"
function setStreamedSampleGroup(streamedSampleId, group) end

--- Set streamed sample volume
---@param streamedSampleId entityId "streamedSampleId"
---@param volume number "volume"
function setStreamedSampleVolume(streamedSampleId, volume) end

--- Set random delay parameters for streamed I3D loading
---@param minDelay number "min delay in seconds"
---@param maxDelay number "max delay in seconds"
function setStreamI3DFileDelay(minDelay, maxDelay) end

--- Set random delay parameters for shared I3D loading
---@param minDelay number "min delay in seconds for loaded shared I3D"
---@param maxDelay number "max delay in seconds for loaded shared I3D"
---@param minDelayCached number "min delay in seconds for cached shared I3D"
---@param maxDelayCached number "max delay in seconds for cached shared I3D"
function setStreamSharedI3DFileDelay(minDelay, maxDelay, minDelayCached, maxDelayCached) end

--- Set fill level mapping for a type (NB: finalizeTerrainFillLayers must be called afterwards)
---@param fillDataPlaneId entityId "fillDataPlaneId"
---@param fillType uint "fillType"
---@param firstFillLevel uint "first fill level to adjust"
---@param firstMappedFillLevel number "mapped fill level corresponding to firstFillLevel"
---@param lastFillLevel uint "last fill level to adjust (optional)"
---@param lastMappedFillLevel number "mapped fill level corresponding to lastFillLevel (optional)"
function setTerrainFillVisualHeight(fillDataPlaneId, fillType, firstFillLevel, firstMappedFillLevel, lastFillLevel, lastMappedFillLevel) end

--- Set terrain height at world pos
---@param terrainId entityId "terrainId"
---@param x number "x"
---@param y number "y (not relevant, can be 0)"
---@param z number "z"
---@param height number "absolute height in m"
function setTerrainHeightAtWorldPos(terrainId, x, y, z, height) end

--- Set the text horizontal alignment to be used for the following renderText calls
---@param alignment integer "one of enum values RenderText.ALIGN_*"
function setTextAlignment(alignment) end

--- Set the text boldness to be used for the following renderText calls
---@param isBold boolean "isBold"
function setTextBold(isBold) end

--- Sets clip area used for text rendering. Characters are only rendered within the clip area.
--- Set to unit area (0,0,1,1) to disable.
---@param clipMinX number
---@param clipMinY number
---@param clipMaxX number
---@param clipMaxY number
function setTextClipArea(clipMinX, clipMinY, clipMaxX, clipMaxY) end

--- Set the text color to be used for the following renderText calls
---@param r number "r"
---@param g number "g"
---@param b number "b"
---@param a number "a"
function setTextColor(r, g, b, a) end

--- Set 3D text rendering depth check
---@param depthCheckEnabled boolean "if true rendered 3D text can be concealed by other meshes"
function setTextDepthTestEnabled(depthCheckEnabled) end

--- Set text first line indentation
---@param indentation number
function setTextFirstLineIndentation(indentation) end

--- Set text line bounds
---@param startLine integer "startLine"
---@param numLines integer "numLines"
function setTextLineBounds(startLine, numLines) end

--- 
---@param lineHeightScale number "factor for adjusting the spacing between lines"
function setTextLineHeightScale(lineHeightScale) end

--- 
---@param rotation number
---@param rotationCenterX float|nil "[optional, default=position of renderText call]"
---@param rotationCenterY float|nil "[optional, default=position of renderText call]"
function setTextRotation(rotation, rotationCenterX, rotationCenterY) end

--- Set the text vertical alignment to be used for the following renderText calls
---@param verticalAlignment integer "one of enum values RenderText.VERTICAL_ALIGN_*"
function setTextVerticalAlignment(verticalAlignment) end

--- Set the text scale width to be used for the following renderText calls
---@param scaleWidth number "scaleWidth"
function setTextWidthScale(scaleWidth) end

--- Set text wrap width
---@param wrapWidth number "wrapWidth"
---@param allowForcedWrap boolean|nil "allow wrapping mid word when no separator is available [optional, default=true]"
function setTextWrapWidth(wrapWidth, allowForcedWrap) end

--- Set translation of a transform object in its local space (relative to parent node)
---@param transformId entityId "transformId"
---@param x number "x translation relative to parent node"
---@param y number "y translation relative to parent node"
---@param z number "z translation relative to parent node"
function setTranslation(transformId, x, y, z) end

--- Set if a trigger should report overlaps with static objects. By default triggers don't report overlaps with static objects
---@param triggerNodeId entityId
---@param reportStatics boolean
function setTriggerReportStatics(triggerNodeId, reportStatics) end

--- Set user attribute value
---@param objectId entityId "objectId"
---@param name string "name"
---@param type integer|nil "Value of UserAttributeType enum [optional]"
---@param value any|nil "value (type must match the type specified in type) [optional]"
function setUserAttribute(objectId, name, type, value) end

--- Set transform object visibility
---@param transformId entityId "transformId"
---@param visibility boolean "visibility"
function setVisibility(transformId, visibility) end

--- Set the day of year condition of the visibility condition (0 to unset)
---@param objectId entityId "objectId"
---@param dayOfYearStart integer "dayOfYearStart"
---@param dayOfYearEnd integer "dayOfYearEnd"
function setVisibilityConditionDayOfYear(objectId, dayOfYearStart, dayOfYearEnd) end

--- Set the minute of day condition (0 to unset) of the visibility condition
---@param objectId entityId "objectId"
---@param minuteOfDayStart integer "minuteOfDayStart"
---@param minuteOfDayEnd integer "minuteOfDayEnd"
function setVisibilityConditionMinuteOfDay(objectId, minuteOfDayStart, minuteOfDayEnd) end

--- Set the render invisible property of the visibility condition
---@param objectId entityId "objectId"
---@param renderInvisible boolean "if true, the object will always be rendered and the custom shader is supposed to change the rendering based on the visibility parameter"
function setVisibilityConditionRenderInvisible(objectId, renderInvisible) end

--- Set the viewerspaciality mask condition of the visibility condition
---@param objectId entityId "objectId"
---@param viewerSpacialityRequiredMask integer
---@param viewerSpacialityPreventMask integer
function setVisibilityConditionViewerSpacialityMask(objectId, viewerSpacialityRequiredMask, viewerSpacialityPreventMask) end

--- Set the shader parameter of the visibility condition
---@param objectId entityId "objectId"
---@param shaderVisibilityParam number "shader parameter when condition is met (ie. object is visible)"
function setVisibilityConditionVisibleShaderParameter(objectId, shaderVisibilityParam) end

--- Set the weather mask condition of the visibility condition
---@param objectId entityId "objectId"
---@param weatherRequiredMask integer
---@param weatherPreventMask integer
function setVisibilityConditionWeatherMask(objectId, weatherRequiredMask, weatherPreventMask) end

--- Set wheel shape direction
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@param dirX number "dirX"
---@param dirY number "dirY"
---@param dirZ number "dirZ"
---@param axleX number "axleX"
---@param axleY number "axleY"
---@param axleZ number "axleZ"
function setWheelShapeDirection(transformId, wheelShapeIndex, dirX, dirY, dirZ, axleX, axleY, axleZ) end

--- Set wheel shape force point
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
function setWheelShapeForcePoint(transformId, wheelShapeIndex, positionX, positionY, positionZ) end

--- Set wheel shape properties
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@param motorTorque number "motorTorque"
---@param brakeTorque number "brakeTorque"
---@param steerAngle number "steerAngle"
---@param rotationDamping number "rotationDamping"
function setWheelShapeProps(transformId, wheelShapeIndex, motorTorque, brakeTorque, steerAngle, rotationDamping) end

--- Set wheel shape steering center
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@param positionX number "positionX"
---@param positionY number "positionY"
---@param positionZ number "positionZ"
function setWheelShapeSteeringCenter(transformId, wheelShapeIndex, positionX, positionY, positionZ) end

--- Set wheel shape tire friction
---@param transformId entityId "transformId"
---@param wheelShapeIndex integer "wheelShapeIndex"
---@param maxLongStiffness number "maxLongStiffness"
---@param maxLatStiffness number "maxLatStiffness"
---@param maxLatStiffnessTireLoad number "maxLatStiffnessTireLoad"
---@param frictionMultiplier number "frictionMultiplier"
function setWheelShapeTireFriction(transformId, wheelShapeIndex, maxLongStiffness, maxLatStiffness, maxLatStiffnessTireLoad, frictionMultiplier) end

--- Set the direction of an object with direction (x y z) and up (upX, upY, upZ) vectors given in world coordinates
---@param transformId entityId "transformId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param upX number "upX"
---@param upY number "upY"
---@param upZ number "upZ"
function setWorldDirection(transformId, x, y, z, upX, upY, upZ) end

--- Set quaternion in world space
---@param objectId entityId "objectId"
---@param x number "x"
---@param y number "y"
---@param z number "z"
---@param w number "w"
function setWorldQuaternion(objectId, x, y, z, w) end

--- Set world rotation of a transform object
---@param transformId entityId "transformId"
---@param x number "x rotation"
---@param y number "y rotation"
---@param z number "z rotation"
function setWorldRotation(transformId, x, y, z) end

--- Set translation of a transform object in world space
---@param transformId entityId "transformId"
---@param wx number "x translation"
---@param wy number "y translation"
---@param wz number "z translation"
function setWorldTranslation(transformId, wx, wy, wz) end

--- Set XML file boolean attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@param value boolean "value"
function setXMLBool(xmlId, attributePath, value) end

--- Set XML file float attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@param value number "value"
function setXMLFloat(xmlId, attributePath, value) end

--- Set XML file integer attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@param value integer "value"
function setXMLInt(xmlId, attributePath, value) end

--- Set XML file string attribute.
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@param value string "value"
function setXMLString(xmlId, attributePath, value) end

--- Set XML file unsigned integer attribute (in the range 0 - 4'294'967'295).
---@param xmlId entityId "xmlId"
---@param attributePath string "attributePath"
---@param value integer "value"
function setXMLUInt(xmlId, attributePath, value) end

--- TODO
---@param shallowWaterSimulation entityId "TODO"
---@param waterGeometry entityId "TODO"
function shallowWaterSimulationAddWaterPlaneGeometry(shallowWaterSimulation, waterGeometry) end

--- Paints a custom solid obstacle in the shape of a rotated rectangle into the simulation. You have to set the number of active obstacles with shallowWaterSimulationSetNumCustomObstacles first.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param obstacleIndex integer "index of the obstacle, starting at 0"
---@param x number "x position of the rectangle center (world coordinates)"
---@param y number "y position of the rectangle center (world coordinates)"
---@param width number "width of the rectangle (world size)"
---@param height number "height of the rectangle (world size)"
---@param angle number "rotation angle of the rectangle (counter-clock wise, in radians)."
---@param velocityX number "x velocity component of the obstacle (0 for a static, non-moving obstacle)"
---@param velocityY number "y velocity component of the obstacle (0 for a static, non-moving obstacle)"
---@param height number "height of the obstacle. makes it so the obstacle only applies when it touches below the water surface. so for a wavey surface, waves can flow under the obstacle if they go low enough."
function shallowWaterSimulationPaintCustomObstacle(shallowWaterSimulation, obstacleIndex, x, y, width, height, angle, velocityX, velocityY, height) end

--- Adds depth into the simulation in the shape of a circle
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param x number "x position of the circle center (world coordinates)"
---@param y number "y position of the circle center (world coordinates)"
---@param radius number "radius of the circle (world size)"
---@param value number "to be added to depth (can be negative for removing water)"
---@param paintHeight float|nil "[optional] height that the paint should happen at. will only be painted if waterHeight in that location >= paintHeight"
function shallowWaterSimulationPaintDepthCircle(shallowWaterSimulation, x, y, radius, value, paintHeight) end

--- Adds depth into the simulation in the shape of a rectangle
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param x number "x position of the rectangle center (world coordinates)"
---@param y number "y position of the rectangle center (world coordinates)"
---@param width number "width of the rectangle (world size)"
---@param height number "height of the rectangle (world size)"
---@param value number "to be added to depth (can be negative for removing water)"
---@param paintHeight float|nil "[optional] height that the paint should happen at. will only be painted if waterHeight in that location >= paintHeight"
function shallowWaterSimulationPaintDepthRect(shallowWaterSimulation, x, y, width, height, value, paintHeight) end

--- Paints a custom depth source into the simulation. Depth sources make it so the depth in that area can never go below the specified value, acting like a source that pours out water until the surrounding area is filled to that level.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param sourceIndex integer "index of the source, starting at 0, up to 16 (currently)"
---@param x number "x position of the rectangle center (world coordinates)"
---@param y number "y position of the rectangle center (world coordinates)"
---@param width number "width of the rectangle (world size)"
---@param height number "height of the rectangle (world size)"
---@param angle number "rotation angle of the rectangle (counter-clock wise, in radians)."
---@param depth number "depth of the water source"
function shallowWaterSimulationPaintDepthSource(shallowWaterSimulation, sourceIndex, x, y, width, height, angle, depth) end

--- Adds velocity into the simulation in the shape of a circle
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param x number "x position of the circle center (world coordinates)"
---@param y number "y position of the circle center (world coordinates)"
---@param radius number "radius of the circle (world size)"
---@param velocityX number "value to be added to the x component of velocity"
---@param velocityY number "value to be added to the y component of velocity"
---@param paintHeight float|nil "[optional] height that the paint should happen at. will only be painted if waterHeight in that location >= paintHeight"
function shallowWaterSimulationPaintVelocityCircle(shallowWaterSimulation, x, y, radius, velocityX, velocityY, paintHeight) end

--- Adds velocity into the simulation in the shape of a rectangle.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param x number "x position of the rectangle center (world coordinates)"
---@param y number "y position of the rectangle center (world coordinates)"
---@param width number "width of the rectangle (world size)"
---@param height number "height of the rectangle (world size)"
---@param velocityX number "value to be added to the x component of velocity"
---@param velocityY number "value to be added to the y component of velocity"
---@param paintHeight float|nil "[optional] height that the paint should happen at. will only be painted if waterHeight in that location >= paintHeight"
function shallowWaterSimulationPaintVelocityRect(shallowWaterSimulation, x, y, width, height, velocityX, velocityY, paintHeight) end

--- TODO
---@param shallowWaterSimulation entityId "TODO"
---@param waterGeometry entityId "TODO"
function shallowWaterSimulationRemoveWaterPlaneGeometry(shallowWaterSimulation, waterGeometry) end

--- Resets simulation to its initial state (water level = determined by the water level planes, all velocities 0)
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
function shallowWaterSimulationResetSimulation(shallowWaterSimulation) end

--- Sets the number of active depth sources.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param numDepthSource integer "number of active depth sources"
function shallowWaterSimulationSetNumDepthSources(shallowWaterSimulation, numDepthSource) end

--- Manually translates the contents of the simulation by integer increments. Only meant for debugging purposes. Setting the world position of the simulation should do this automatically.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param x integer "x offset to translate the simulation contents by"
---@param y integer "y offset to translate the simulation contents by"
function shallowWaterSimulationTranslateSimulation(shallowWaterSimulation, x, y) end

--- Enable/disable physics simulation
---@param state boolean "state"
function simulatePhysics(state) end

--- Time scale of physics simulation
---@param scale number "scale"
function simulatePhysicsTimeScale(scale) end

--- Source script file
---@param filename string "filename"
---@param environment ref|nil "environment [optional]"
function source(filename, environment) end

--- Splits the given light source from its merged shadow, if the light sources' shadow is merged with other light sources' shadows
---@param lightId entityId
function splitLightShadow(lightId) end

--- Attach to debugger
--- Note that the connection is not instant and breakpoints may only trigger a few frames later

function startDebugging() end

--- Detach from debugger

function stopDebugging() end

--- Stop sample object
---@param sampleId entityId "sampleId"
---@param delay number "delay until to stop playing [ms]"
---@param fadeTime number "time to fade out the sample [ms]"
function stopSample(sampleId, delay, fadeTime) end

--- Stop streamed sample
---@param streamedSampleId entityId "streamedSampleId"
function stopStreamedSample(streamedSampleId) end

--- Stream align read to byte boundary
---@param streamId entityId "streamId"
function streamAlignReadToByteBoundary(streamId) end

--- Stream align write to byte boundary
---@param streamId entityId "streamId"
function streamAlignWriteToByteBoundary(streamId) end

--- Stream get number of unread bits
---@param streamId entityId "streamId"
---@return value integer "value"
function streamGetNumOfUnreadBits(streamId) end

--- Stream get read offset it bits
---@param streamId entityId "streamId"
---@return offset integer "offset"
function streamGetReadOffset(streamId) end

--- Return the write pointer offset in bits
---@param streamId entityId "streamId"
---@return offset integer "offset"
function streamGetWriteOffset(streamId) end

--- Stream I3D file asynchronously
---@param filename string "filename"
---@param callbackFunctionName string|nil "callback(nodeId, LoadI3DFailedReason.* failedReason, arguments) [optional]"
---@param callbackTarget object|nil "target [optional]"
---@param arguments object|nil "arguments to return in callback function [optional]"
---@param addPhysics boolean|nil "addPhysics [optional]"
---@param callOnCreate boolean|nil "callOnCreate [optional]"
---@param verbose boolean|nil "verbose [optional]"
---@return requestId integer "request id for streaming, used to cancel the stream request"
function streamI3DFile(filename, callbackFunctionName, callbackTarget, arguments, addPhysics, callOnCreate, verbose) end

--- Stream read boolean
---@param streamId entityId "streamId"
---@return value boolean "value"
function streamReadBool(streamId) end

--- Stream read 32bit float
---@param streamId entityId "streamId"
---@return value float "value"
function streamReadFloat32(streamId) end

--- Stream read 16bit signed integer
---@param streamId entityId "streamId"
---@return value integer "value"
function streamReadInt16(streamId) end

--- Stream read 32bit signed integer
---@param streamId entityId "streamId"
---@return value integer "value"
function streamReadInt32(streamId) end

--- Stream read 8bit signed integer
---@param streamId entityId "streamId"
---@return value integer "value"
function streamReadInt8(streamId) end

--- Stream read 32bit signed integer
---@param streamId entityId "streamId"
---@param numberofBits integer "numberofBits"
---@return value integer "value"
function streamReadIntN(streamId, numberofBits) end

--- Stream read manual timestamp
---@param streamId entityId "streamId"
---@return timestamp integer "timestamp"
function streamReadManualTimestamp(streamId) end

--- Stream read 32bit float
---@param streamId entityId "streamId"
---@return value string "value"
function streamReadString(streamId) end

--- Stream write 16-bit unsigned integer.
---@param streamId entityId "streamId"
---@return uint16 integer "uint16"
function streamReadUInt16(streamId) end

--- Stream read 32bit unsigned integer
---@param streamId entityId "streamId"
---@return value integer "value"
function streamReadUInt32(streamId) end

--- Stream read 8-bit unsigned integer.
---@param streamId entityId "streamId"
---@return uint8 integer "uint8"
function streamReadUInt8(streamId) end

--- Stream write N-bit unsigned integer. 0&lt;N&lt;32.
---@param streamId entityId "streamId"
---@param numberOfBits integer "numberOfBits"
---@return value integer "value"
function streamReadUIntN(streamId, numberOfBits) end

--- Set the read pointer to the given offset in bits
---@param streamId entityId "streamId"
---@param offset integer "offset"
function streamSetReadOffset(streamId, offset) end

--- Set the write pointer to the given offset in bits
---@param streamId entityId "streamId"
---@param offset integer "offset"
function streamSetWriteOffset(streamId, offset) end

--- Stream shared I3D file. Can call the callback in the same callstack when the file is already loaded
---@param filename string "filename"
---@param callbackFunctionName string|nil "callback(nodeId, LoadI3DFailedReason.* failedReason, arguments) [optional]"
---@param callbackTarget object|nil "target [optional]"
---@param arguments object|nil "arguments to return in callback function [optional]"
---@param addPhysics boolean|nil "addPhysics [optional]"
---@param callOnCreate boolean|nil "callOnCreate [optional]"
---@param verbose boolean|nil "verbose [optional]"
---@return requestId integer "request id for streaming, used to cancel the stream request"
function streamSharedI3DFile(filename, callbackFunctionName, callbackTarget, arguments, addPhysics, callOnCreate, verbose) end

--- Stream write boolean
---@param streamId entityId "streamId"
---@param value boolean "value"
---@return value boolean "value"
function streamWriteBool(streamId, value) end

--- Stream write 32bit float
---@param streamId entityId "streamId"
---@param value number "value"
function streamWriteFloat32(streamId, value) end

--- Stream write 16bit signed integer
---@param streamId entityId "streamId"
---@param value integer "value"
function streamWriteInt16(streamId, value) end

--- Stream write 32bit signed integer
---@param streamId entityId "streamId"
---@param value integer "value"
function streamWriteInt32(streamId, value) end

--- Stream write 8bit signed integer
---@param streamId entityId "streamId"
---@param value integer "value"
function streamWriteInt8(streamId, value) end

--- Stream write N bit signed integer. 0 &lt; N &lt; 32.
---@param streamId entityId "streamId"
---@param value integer "value"
---@param numberOfBits integer "numberOfBits"
function streamWriteIntN(streamId, value, numberOfBits) end

--- Stream write manual timestamp
---@param streamId entityId "streamId"
---@param timestamp integer "timestamp"
function streamWriteManualTimestamp(streamId, timestamp) end

--- Stream write stream
---@param streamId entityId "streamId"
---@param streamSrcId entityId "streamSrcId"
---@param numBits integer "numBits"
---@param useReadStream boolean "useReadStream"
function streamWriteStream(streamId, streamSrcId, numBits, useReadStream) end

--- Stream write string
---@param streamId entityId "streamId"
---@param value string "value"
function streamWriteString(streamId, value) end

--- Stream write timestamp
---@param streamId entityId "streamId"
function streamWriteTimestamp(streamId) end

--- Stream write 16-bit unsigned integer.
---@param streamId entityId "streamId"
---@param value integer "value"
function streamWriteUInt16(streamId, value) end

--- Stream write 32bit unsigned integer
---@param streamId entityId "streamId"
---@param value integer "value"
function streamWriteUInt32(streamId, value) end

--- Stream write 8-bit unsigned integer.
---@param streamId entityId "streamId"
---@param uint8 integer "uint8"
function streamWriteUInt8(streamId, uint8) end

--- Stream write N-bit unsigned integer. 0&lt;N&lt;32.
---@param streamId entityId "streamId"
---@param value integer "value"
---@param numberofBits integer "numberofBits"
function streamWriteUIntN(streamId, value, numberofBits) end

--- Project world position to view space for specified camera
---@param cameraId entityId "cameraId"
---@param x number
---@param y number
---@param z number
---@return x float "VS"
function transformToViewSpace(cameraId, x, y, z) end

--- Converts an unicode value to an utf8 string
---@param unicode integer "unicode"
---@return utf8string string "utf8string"
function unicodeToUtf8(unicode) end

--- Unlink node from parent
---@param objectId entityId "objectId"
function unlink(objectId) end

--- Unpin a shared I3D file in the cache, so it can be auto-deleted again
---@param filename string "filename"
function unpinSharedI3DFileInCache(filename) end

--- Transform vector from screen space into world space
---@param sx number "screenspace x [0..1]"
---@param sy number "screenspace y [0..1]"
---@param sz number "screenspace z / depth [0..1]"
---@return wx float "wx"
function unProject(sx, sy, sz) end

--- Update
---@param conditionalAnimationEntityId entityId "conditionalAnimationEntityId"
---@param dt number "dt"
function updateConditionalAnimation(conditionalAnimationEntityId, dt) end

--- update differential
---@param transformId entityId "transformId"
---@param index integer "index"
---@param ratio number "ratio"
---@param bias number "bias"
function updateDifferential(transformId, index, ratio, bias) end

--- Requests the simulation to be updated by the given time amount. The internals automatically take care of running the correct number of iterations (based on the delta time set in shallowWaterSimulationSetParameters) per requested update.
--- It's intended that you just forward the delta time from the script update function here, but you can also scale it to simulate fast forward or slow down of the simulation time.
---@param shallowWaterSimulation entityId "id of the shallow water simulation"
---@param simulatedTime number "simulated time in seconds."
function updateShallowWaterSimulation(shallowWaterSimulation, simulatedTime) end

--- Returns the length of an utf8 formated string
---@param utf8string string "utf8string"
---@return length integer "length"
function utf8Strlen(utf8string) end

--- Returns a sub string of an utf8 formated string
---@param utf8string string "utf8string"
---@param startIndex integer "startIndex"
---@param length integer|nil "length [optional]"
---@return subString string "subString"
function utf8Substr(utf8string, startIndex, length) end

--- Returns a lower case string of an utf8 formated string
---@param utf8string string "utf8string"
---@return utf8string string "utf8string"
function utf8ToLower(utf8string) end

--- Converts an utf8 string to unicode
---@param utf8string string "utf8string"
---@return unicode integer "unicode"
function utf8ToUnicode(utf8string) end

--- Return a upper case string of an utf8 formated string
---@param utf8string string "utf8string"
---@return utf8string string "utf8string"
function utf8ToUpper(utf8string) end

--- Remove voice chat connection
---@param connectionId integer "connectionId"
function voiceChatRemoveConnection(connectionId) end

--- World space to local space transformation, only direction without translation
---@param transformId entityId "transformId"
---@param wdx number "world direction x"
---@param wdy number "world direction y"
---@param wdz number "world direction z"
---@return ldx float "local direction x"
function worldDirectionToLocal(transformId, wdx, wdy, wdz) end

--- World space to local space rotation transformation
---@param transformId entityId "transformId"
---@param wrx number "world rotation x"
---@param wry number "world rotation y"
---@param wrz number "world rotation z"
---@return lrx float "local rotation x"
function worldRotationToLocal(transformId, wrx, wry, wrz) end

--- World space to local space translation transformation
---@param transformId entityId "transformId"
---@param wx number "world x"
---@param wy number "world y"
---@param wz number "world z"
---@return lx float "local x"
function worldToLocal(transformId, wx, wy, wz) end

--- Write fill plane surface information to stream
---@param fillPlaneShapeId entityId "fillPlaneShapeId"
---@param streamId entityId "streamId"
---@return success boolean
function writeFillPlaneToStream(fillPlaneShapeId, streamId) end

