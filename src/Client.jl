"""
TESTIFY
"""
module Client

using Dates
using LinearAlgebra # for dot etc...
using Distributed

using cywwtaip  # HTW lib from AI professor

####################################################################################################

outputBacktrace(ex) = open(FILE_ERROR,"w+") do f Base.showerror(f, ex, catch_backtrace()) end

const ROOT = abspath(joinpath(@__DIR__,"../"))
const LOGS = abspath(joinpath(ROOT,"logs/"))
const FILE_ERROR = abspath(joinpath(LOGS, "error.log"))

name = nothing
winMsg  = nothing
server = nothing
client = nothing

latency = 0
playerNumber = 1 # 0 = nobody, 1 - 3 a player

mutable struct AIBot
  parent::Bot
  node::Union{Nothing, GraphNode}
  prevnode::Union{Nothing, GraphNode}
  target::Union{Nothing, GraphNode}
  targetEnergyField::Union{Nothing, Array{Float32, 1}}
  history::Array{GraphNode,1}
  avoid::Array{GraphNode,1}
  movespeed::Float32

  AIBot(parent::Bot, node::Union{Nothing, GraphNode}) = new(parent,node,nothing,nothing,nothing,[],[],0)
end

####################################################################################################

getPlayerbotsTemplate() = Array{Union{Nothing,Array{Union{Nothing,Bot},1}},1}(undef,3)
getBotsTemplate() = Array{Union{Nothing,Bot},1}(undef,3)
getAIBotsTemplate() = Array{Union{Nothing,AIBot},1}(undef,3)

world = getWorldTemplate()
bots=getBotsTemplate()
playerbots=getPlayerbotsTemplate()
AIBots = getAIBotsTemplate()

worldlist=Dict{Symbol,Integer}()
listOfEnergyFields = Array{Array{Float32,1},1}()

CHARGED = 0
BATTERY_LOW = false
ENERGY = 0
MAXMOVESPEED = 0

####################################################################################################

function resetWorld()
  open(FILE_ERROR,"w+") do f write(f,"") end

  global world, worldlist, playerbots, bots, AIBots
  world = getWorldTemplate()
  playerbots = getPlayerbotsTemplate()
  bots = getBotsTemplate()
  worldlist = Dict{Symbol,Integer}()
  AIBots = getAIBotsTemplate()

  fill!(world,nothing)
  fill!(playerbots,nothing)
  fill!(bots,nothing)
  fill!(AIBots,nothing)

  global listOfEnergyFields = generateListOfEnergyFields()
  global MAXMOVESPEED = 0
end

toKey(list::AbstractArray) = Symbol(join(list, ','))

function getNode(position::Array{Float32,1})
  key = toKey(position)
  haskey(worldlist, key) ? world[worldlist[key]] : nothing
end

####################################################################################################

function generateListOfEnergyFields()
  return [Float32[-0.95,0,0]] #[Float32[0.95,0,0],Float32[0,0.95,0],Float32[0,0,0.95]] #,Float32[-0.95,0,0],Float32[0,-0.95,0],,Float32[0,0,-0.95]]

  i=0; r = Array{Array{Float32,1},1}() # minus zero field
  for l=0:1
    for k=0:1
      for j=0:1
        #v = nothing
        if j == k == l == 0 continue end # skip zeros
        i+=1
        push!(r,Float32[j,k,l])
      end
    end
  end
  r
end

####################################################################################################

distance(p1,p2) = √((p2[1] - p1[1])^2 + (p2[2] - p1[2])^2 + (p2[3] - p1[3])^2)

####################################################################################################
# https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
function getAngle(Va,Vb,Vn=[0,1,0])::Float32
  #angle = atan(Vn ⋅ (Va × Vb), Va ⋅ Vb)
  #angle = atan(Vn ⋅ (Va × Vb) / (Va ⋅ Vb))
  #return angle

  vcross = Va × Vb
  vdot = Va ⋅ Vb
  sign = Vn ⋅ vcross

  normed = norm(Va) * norm(Vb)
  sina = norm(vcross) / normed
  cosa = vdot / normed
  angle = atan(sina, cosa)
  if sign<0 angle=-angle end

  angle
end

# http://geomalgorithms.com/a05-_intersect-1.html
# https://stackoverflow.com/questions/5666222/3d-line-plane-intersection
function getIntersectionPlanePoint(pos,target,up)::Array{Float32,1}
  n = up
  V0 = pos
  P0 = target
  P1 = (target+n)
  w = P0 - V0 # ok
  u = P1 - P0 # ok
  s = -(n ⋅ w) / (n ⋅ u) # ok
  p = P0 + s*u # ok
end

function localAngleToTarget(pos,target,forward=[0,0,1])::Float32 #
  global_up = up = normalize(pos)
  #right = forward × global_up
  #up = right × forward #
  #right = (forward × up) #* -1
  intersect = getIntersectionPlanePoint(pos,target,up)
  #dist = distance(pos,target)
  #mul = dist < 1 ? 1 - 1/dist : 1
  getAngle(pos,intersect,forward*-1)
end

function moveAI(client::NetworkClient, aibot::AIBot)
  target = aibot.target == nothing ? aibot.prevnode.position : aibot.target.position # aibot.targetEnergyField
  dist = distance(aibot.parent.position,target)
  #if dist < 0.051 println("$(aibot.parent.id): $(dist)") end
  angle = localAngleToTarget(aibot.parent.position, target, aibot.parent.direction)
  #println("$(aibot.parent.id): "*string(angle))
  #println("$(aibot.parent.id): $(aibot.parent.speed): $(aibot.movespeed)")
  changeMoveDirection(client, aibot.parent.id, angle*10*pi)
end

#speed = getBotSpeed(client)
#score = getScore(client, playerNumber)
#position = getBotPosition(client, 0, 0)
#direction = getBotDirection(client, 0)

#println("Speed: $speed")
#println("Score: $score")
#println("Position: $position")
#println("Direction: $direction")

#changeMoveDirection(client, 1, -0.08f0)

# a star algorithmn

#function getDirectionXXX()
  #bots[1].direction
  #position - prev_position
  #dir_pos  = target - position
  #changeMoveDirection(client, 1, -0.08f0)
#end

function getWayPoints()
  i=0; for graph in world
    i+=1
    i += 1; j=0;
    #parse(graph.neighbors,(node)->begin
    #  j += 1
    #  if node.owner > 2 println("[$i,$j]: "*toInfoString(node)) end
    #  if node.owner < 0 println("###### [$i,$j]: "*toInfoString(node)) end
    #end)
  end
end

####################################################################################################

upgradeToAIBots(bots::Array{Bot,1},nodes::Array{GraphNode,1}) = global AIBots = [AIBot(bot,node) for (bot,node) in zip(bots,nodes)]

function updateBots(client::NetworkClient)
  global BATTERY_LOW, ENERGY, CHARGED, MAXMOVESPEED
  #cywwtaip.updateBots(client)
  #cywwtaip.updateBots(client, bots)
  ENERGY = 1
  CHARGED = 0

  for aibot in AIBots
    prevposition = aibot.parent.position
    cywwtaip.updateBot(client, aibot.parent)
    aibot.movespeed = min(norm(prevposition - aibot.parent.position),1)

    if aibot.parent.id  == 1
      #if aibot.movespeed > MAXMOVESPEED MAXMOVESPEED = aibot.movespeed end
      #ENERGY = (aibot.movespeed / MAXMOVESPEED)

      if (!BATTERY_LOW && ENERGY < 0.5) || (BATTERY_LOW && ENERGY == 1)
        CHARGED = ENERGY == 1 ? 1 : -1
        BATTERY_LOW = !BATTERY_LOW
      end
    end

    searchCurrentNode(aibot)
    #aibot.node=getNode(aibot.parent.position)
    if aibot.node == nothing @warn "bot $(aibot.parent.id): no start node found!" end
    searchNext(aibot)
    moveAI(client, aibot)
  end
end

function saveClosestNode(bots, nodes, current)
  for bot in bots
    prev = nodes[bot.id]
    if prev == nothing || (prev != nothing && distance(bot.position,current.position) < distance(bot.position, prev.position))
      nodes[bot.id] = current
    end
  end
end

function createWorld(client::NetworkClient)
  global world, worldlist, bots, playerbots

  bots = cywwtaip.getBots(client, playerNumber)
  nodes = Union{Nothing,GraphNode}[nothing,nothing,nothing]

  world = getGraph(client)
  if length(world) <= 0 return end

  # save to worldlist
  i=0; for node in world
    i+=1
    node.id=i
    worldlist[toKey(node.position)] = i
    #node.position = cywwtaip.getPosition(node.handle)
    saveClosestNode(bots, nodes, node)
  end

  # replace neighbors
  for parent in world
    j=0; for node in parent.neighbors
      j+=1
      if isa(node,cywwtaip.JGraphNode) # only replace if type is wrong
        parent.neighbors[j] = getNode(getPosition(node))
      end
    end
  end

  for node in nodes
    if node == nothing error("Start Nodes not complete!") end
  end

  #playerbots = getBots(client)
  upgradeToAIBots(bots,Array{GraphNode,1}(nodes))
end

function searchCurrentNode(aibot::AIBot)
  (old,best,bestdist) = (nothing,aibot.node, distance(aibot.node.position,aibot.parent.position))

  for node in world
    dist = distance(node.position,aibot.parent.position)
    if dist < bestdist; (old,best,bestdist) = (best,node,dist); end
  end

#=
  while true
    found = false
    for node in best.neighbors
      dist = distance(node.position,aibot.parent.position)
      if dist < bestdist
        if old != nothing && old == node best=old; break end
        (old,best,bestdist) = (best,node,dist)
        found = true
      end
    end
    if !found break end
  end
=#

  aibot.node = best
  if aibot.prevnode == nothing aibot.prevnode = best end
end

function recharge(aibot::AIBot)
  #if aibot.parent.id != 1 return end
  #if CHARGED < 0 aibot.targetEnergyField = nothing end
  #if !BATTERY_LOW return end
  if aibot.targetEnergyField != nothing
    #if distance(aibot.parent.position,aibot.targetEnergyField) < 0.2
    #  aibot.targetEnergyField = nothing
    #else
    #  return
    #end
  end

  best = (2,Float32[0,0,0])
  #for energyField in listOfEnergyFields
  #  x = distance(aibot.parent.position,energyField)
  #  if x < best[1] best=(x,energyField) end
  #end

  #println("$i: $(join(bot.position, ", ")) : $(join(best[2], ", "))")
  aibot.targetEnergyField = listOfEnergyFields[1] #best[2]
end

function searchNext(aibot::AIBot)
  recharge(aibot)
  if aibot.node == nothing return end
  #if aibot.parent.id != 1 aibot.target=aibot.node; return end
  #return
  #if !charge
    if aibot.target != nothing && aibot.target.owner == aibot.parent.id aibot.target = nothing end
    #distance(aibot.parent.position,aibot.target.position) < 0.051 #
    if aibot.target == nothing
      (best,bestdist)=(nothing,999)
      for node in aibot.node.neighbors
        if aibot.parent.id != 1 && node.blocked continue end # skip blocked
        if node.owner == aibot.parent.id continue end # skip owned
        if in(node,aibot.avoid) || in(node,aibot.history) continue end
        if aibot.parent.id != 1 best=node
        else
          x = distance(node.position,aibot.targetEnergyField)
          if x < bestdist; (best,bestdist)=(node,x); end
        end
      end
      if best != nothing
        push!(aibot.history,aibot.node)
        aibot.target = best
      else
        if !in(aibot.node,aibot.avoid) push!(aibot.avoid,aibot.node) end
        if length(aibot.history)>1
          pop!(aibot.history)
          aibot.target = aibot.history[end]
        else
          @warn "$(aibot.parent.id) is stuck on $(aibot.node.position)!"
        end
      end
    end
  #end
end

function updateWorld(client::NetworkClient)
  global world, listOfEnergyFields, bsave
  n=1+1
  updateGraph(client, world)
  updateBots(client)
end

function printWorld(printer::Union{Nothing,Function})
  if printer == nothing return end

str="""
# $(replace(string(now()),"T"=>" "))
-----------------------------------------
Position
 1:[$(string(join(bots[1].position,", ")))]
 2:[$(string(join(bots[2].position,", ")))]
 3:[$(string(join(bots[3].position,", ")))]

Direction
 1:[$(string(join(bots[1].direction,", ")))]
 2:[$(string(join(bots[2].direction,", ")))]
 3:[$(string(join(bots[3].direction,", ")))]

Speed
 1:[$(string(bots[1].speed))]
 2:[$(string(bots[2].speed))]
 3:[$(string(bots[3].speed))]
-----------------------------------------
"""
  @sync @distributed for node in world str*=toInfoString(node)*"\n" end
  str*="_________________________________________\n"

  printer(str)
end

####################################################################################################
using Revise
function main(args::Array{String,1}; printer::Union{Nothing,Function}=nothing)
  println("--------------------------------------------------------------------------------")
  @info "Start Game..."
  global playerNumber, latency

  len = length(args)
  global server = len>1 ? args[1] : "localhost"
  global name = len>2 ? args[2] : "MyTeam"
  global winMsg = len>3 ? args[3] : "GEWONNEN!"
  global client = nothing

  # Time limits
  serverTimeout = 10
  timeLimit = 60 # 60s
  timeLimitBeforeStart = 10 # 60s
  latency = 0

  timeLimitBroke = false
  prev_position = nothing
  startTime = 0
  tickTime = 0

  lstart=0
  lend=0

  resetWorld()

  try
    cywwtaip.init()
    cywwtaip.reset()

    @info "Connect to server '$server' (Timeout in $serverTimeout seconds)..."
    client = NetworkClient(server, name, winMsg, serverTimeout)

    playerNumber = getMyPlayerNumber(client)
    createWorld(client) # create world first

    @info "Wait for start..."
    startTime = time()
    while isAlive(client)
      currentTime = (time() - startTime)

      lstart = time()
      position = getBotPositions(client, playerNumber)
      lend = time()

      if prev_position != nothing && prev_position != position break end
      prev_position = position

      if (time() - tickTime) > 1
        println("Seconds: $(timeLimitBeforeStart - round(currentTime))")
        tickTime=time()
      end

      if currentTime > timeLimitBeforeStart timeLimitBroke=true; break end # auto close
    end

    latency = lend-lstart
    startTime = lstart-1
    tickTime=0

    if !timeLimitBroke
      @info "Main Loop..."
      println("\n[START]\n------------------------------")

      while isAlive(client)
        currentTime = (time() - startTime)

        if (time() - tickTime) > 1
          #println("Seconds: $(timeLimit - round(currentTime)), latency: $latency\n")
          tickTime=time()
          #printWorld(printer) # 0.3
        end

        if currentTime > timeLimit timeLimitBroke=true; break end #after Xs break this loop

        updateWorld(client)
      end
    end

  catch ex
    outputBacktrace(ex)
    error("NetworkClient error! For more details look in $FILE_ERROR")
  end

  print("Game ended.")
  #cywwtaip.destroy() # cleanup cywwtaip
end

end # Client
