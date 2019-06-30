"""
TESTIFY
"""
module Client

using Dates
using LinearAlgebra # for dot etc...
using Distributed
using DataStructures

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
  startnode::Union{Nothing, GraphNode}
  prevnode::Union{Nothing, GraphNode}
  target::Union{Nothing, GraphNode}
  targetEnergyField::Union{Nothing, GraphNode}
  movespeed::Float32

  AIBot(parent::Bot, node::Union{Nothing, GraphNode}) = new(parent,node,nothing,nothing,nothing,nothing,0)
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
listOfEnergyNodes = Array{GraphNode,1}()
listOfEnergyFields = Array{GraphNode,1}()
listOfWaypoints= Array{Array{GraphNode,1},1}()

ENERGYTIME = 0
ENERGYAREA = 0.94

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

  global listOfEnergyNodes = Array{GraphNode,1}()
  global listOfEnergyFields = Array{GraphNode,1}()
  global listOfWaypoints= [GraphNode[],GraphNode[],GraphNode[]]
end

toKey(list::AbstractArray) = Symbol(join(list, ','))

getNode(id::Integer) = id>0 && id<=length(world) ? world[id] : nothing

function getNode(position::Array{Float32,1})
  key = toKey(position)
  haskey(worldlist, key) ? getNode(worldlist[key]) : nothing
end

####################################################################################################

distance(p1::Position,p2::Position) = √((p2[1] - p1[1])^2 + (p2[2] - p1[2])^2 + (p2[3] - p1[3])^2)
distance(p1::GraphNode,p2::GraphNode) = distance(p1.position,p2.position)
distance(p1::GraphNode,p2::Position) = distance(p1.position,p2)
distance(p1::Position,p2::GraphNode) = distance(p1,p2.position)

# https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
# https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
function getAngle(Va::Position,Vb::Position,Vn=[0,1,0]; mul=1)::Float32
  #angle = atan(Vn ⋅ (Va × Vb), Va ⋅ Vb)
  #angle = atan(Vn ⋅ (Va × Vb) / (Va ⋅ Vb))
  #return angle

  vcross = Va × Vb
  vdot = (Va ⋅ Vb) * (1.0/mul)
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
function getIntersectionPlanePoint(pos::Position,target::Position,up::Position)::Array{Float32,1}
  n = up
  V0 = pos
  P0 = target
  P1 = (target+n)
  w = P0 - V0 # ok
  u = P1 - P0 # ok
  s = -(n ⋅ w) / (n ⋅ u) # ok
  p = P0 + s*u # ok
end

function localAngleToTarget(pos::Position,target::Position,forward::Position=[0,0,1]; mul=1)::Float32
  global_up = up = normalize(pos)
  #right = forward × global_up
  #up = right × forward #
  #right = (forward × up) #* -1
  intersect = getIntersectionPlanePoint(pos,target,up)
  getAngle(pos,intersect,forward*-1; mul=mul)
end

####################################################################################################

function moveAI(client::NetworkClient, aibot::AIBot)
  target = aibot.target == nothing ? aibot.startnode : aibot.target # aibot.targetEnergyField
  dist = distance(aibot.parent.position,target)
  #if dist < 0.051 println("$(aibot.parent.id): $(dist)") end
  # 0.024, 0.016, 0.01
  mul = ((1000 * aibot.parent.speed)/dist)
  if mul < 1 mul=1 end
  angle = localAngleToTarget(aibot.parent.position, target.position, aibot.parent.direction; mul=mul)
  #println("$(aibot.parent.id): "*string(angle))
  #println("$(aibot.parent.id): $(aibot.parent.speed): $(aibot.movespeed)")
  changeMoveDirection(client, aibot.parent.id, angle)
end

####################################################################################################

upgradeToAIBots(bots::Array{Bot,1},nodes::Array{GraphNode,1}) = global AIBots = [AIBot(bot,node) for (bot,node) in zip(bots,nodes)]

function updateBots(client::NetworkClient)
  #cywwtaip.updateBots(client)
  #cywwtaip.updateBots(client, bots)

  for aibot in AIBots
    prevposition = aibot.parent.position
    start=time()
    cywwtaip.updateBot(client, aibot.parent)
    latency=time()-start
    aibot.movespeed = min(norm(prevposition - aibot.parent.position),1)
    aibot.parent.position += aibot.parent.direction*latency # adjust to future position because we have latency

    searchCurrentNode(aibot)
    #aibot.node=getNode(aibot.parent.position)
    if aibot.node == nothing @warn "bot $(aibot.parent.id): no start node found!" end
    if aibot.startnode == nothing aibot.startnode = aibot.node end
    searchNext(aibot)
    moveAI(client, aibot)
  end
end

function saveClosestNode(bots, nodes, current)
  for bot in bots
    prev = nodes[bot.id]
    if prev == nothing || (prev != nothing && distance(bot.position,current) < distance(bot.position, prev))
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

  f=world[1]
  bX=f; sX=f; bY=f; sY=f; bZ=f; sZ=f

  # save to worldlist
  i=0; for node in world
    i+=1
    node.id=i
    node.position
    worldlist[toKey(node.position)] = i
    (x,y,z) = node.position

    # energy fields with abs max x,y or z
    if bX.position[1] < x bX=node end
    if sX.position[1] > x sX=node end
    if bY.position[2] < y bY=node end
    if sY.position[2] > y sY=node end
    if bZ.position[3] < z bZ=node end
    if sZ.position[3] > z sZ=node end

      # all energy field nodes
    if ((x>y&&x>z) && x>ENERGYAREA) || ((y>x&&y>z) && y>ENERGYAREA) || ((z>y&&z>x) && z>ENERGYAREA)
      push!(listOfEnergyNodes,node)
    end

    #node.position = cywwtaip.getPosition(node.handle)
    saveClosestNode(bots, nodes, node)
  end

  # save energy fields with abs max x,y or z
  push!(listOfEnergyFields,bX); push!(listOfEnergyFields,sX)
  push!(listOfEnergyFields,bY); push!(listOfEnergyFields,sY)
  push!(listOfEnergyFields,bZ); push!(listOfEnergyFields,sZ)

  # replace neighbors with GraphNodes
  for parent in world
    j=0; for node in parent.neighbors
      j+=1
      if isa(node,cywwtaip.JGraphNode) # only replace if type is wrong
        node = getNode(getPosition(node))
        node.parent = parent
        parent.neighbors[j] = node
      end
    end
  end

  for node in nodes
    if node == nothing error("Start Nodes not complete!") end
  end

  #playerbots = getBots(client)
  upgradeToAIBots(bots,Array{GraphNode,1}(nodes))
end

####################################################################################################

function searchCurrentNode(aibot::AIBot)
  (old,best,bestdist) = (nothing,aibot.node, distance(aibot.node,aibot.parent.position))

  for node in world
    dist = distance(node,aibot.parent.position)
    if dist < bestdist; (old,best,bestdist) = (best,node,dist); end
  end

  aibot.node = best
  if aibot.prevnode == nothing aibot.prevnode = best end
end

function reconstruct_path!(cameFrom::AbstractDict, current::GraphNode)
  total_path = []
  for (_,node) in cameFrom
    push!(total_path,node)
  end
  push!(total_path,current)
  total_path
end

function getValue(set::SortedSet,key)
  for (fvalue,fkey) in set
    if fkey == key return fvalue end
  end
  nothing
end

function setValue!(set::SortedSet, key, value)
  rm=nothing
  for t in set
    (fvalue,fkey)=t
    if fkey == key
      if fvalue != value
        rm=t
      else
        return false # same entry?
      end
      break
    end
  end
  if rm != nothing delete!(set,rm) end
  push!(set,(value,key))
  true
end

# still looking for a perfect solution here...
function clustering()
 # energy field are already defined in the code
 # i calculate the distance at runtime, but i see now a pre-calculation with nodes might be better here
 # i have already a search node function with gets me the closest node for a bot
 # i need to connect clustering and aStarSearch somehow, to make it faster
 # position of a bot is already in relation with latency
end

# filter not owned fields and non blocked fields
filterFreeFields() = filter((x)->!x.owner.blocked && x.owner!=playerNumber,world)

function aStarSearch(start::GraphNode, goal::GraphNode)
  # The set of nodes already evaluated
  closedSet = GraphNode[]

  # The set of currently discovered nodes that are not evaluated yet.
  # Initially, only the start node is known.
  openSet = GraphNode[start]

  # For each node, which node it can most efficiently be reached from.
  # If a node can be reached from many nodes, cameFrom will eventually contain the
  # most efficient previous step.
  cameFrom = OrderedDict{Int32,GraphNode}() # an empty map
  #cameFrom[start.id] = nothing

  # For each node, the cost of getting from the start node to that node.
  gScore = SortedSet{Tuple{Float32,Int32}}() # map with default value of Infinity

  # The cost of going from start to start is zero.
  setValue!(gScore, start.id, 0)

  # For each node, the total cost of getting from the start node to the goal
  # by passing by that node. That value is partly known, partly heuristic.
  fScore = SortedSet{Tuple{Float32,Int32}}() # map with default value of Infinity

  # For the first node, that value is completely heuristic.
  setValue!(fScore, start.id, distance(start, goal)) # heuristic_cost_estimate = distance

  for i=1:length(world) #while length(openSet)>0 # avoid endless loop
    #sort!(s,alg=InsertionSort,lt=(x,y)->x[2]<y[2])
    current = getNode(first(fScore)[2]) #the node in openSet having the lowest fScore[] value
    if current == goal
      return reconstruct_path!(cameFrom, current)
    end

    openSet = filter!(e->e ≠ current,openSet) # delete
    #delete!(openSet,current)
    push!(closedSet, current)

    for neighbor in current.neighbors
      if in(neighbor,closedSet) || neighbor.blocked || neighbor.owner != playerNumber continue end # Ignore the neighbor which is already evaluated or blocked.

      # The distance from start to a neighbor
      tentative_gScore = getValue(gScore,current.id) + distance(current, neighbor) # dist_between = distance
      nscore = getValue(gScore,neighbor.id)

      if !in(neighbor,openSet) #neighbor not in openSet	# Discover a new node
        push!(openSet, neighbor)
      elseif nscore != nothing && tentative_gScore >= nscore
        continue
      end

      # This path is the best until now. Record it!
      cameFrom[neighbor.id] = current
      setValue!(gScore, neighbor.id, tentative_gScore)
      setValue!(fScore, neighbor.id, tentative_gScore + distance(neighbor, goal)) # heuristic_cost_estimate = distance
    end
  end
  list = reconstruct_path!(cameFrom, goal)
  #@warn "a*-search result $(length(list)) -> $(in(goal,list))"
  list
end

####################################################################################################

function rechargeWhenLow(aibot)
  #if (ENERGYTIME-time()) >= 9 return false end
  if aibot.targetEnergyField != nothing return true end

  best = (999,nothing)
  for node in listOfEnergyFields
    #node = getNode(index)
    dist = distance(aibot.node, node) #
    if best[1] > dist best=(dist,node) end
  end
  println(best[2].position)
  aibot.targetEnergyField = best[2] #rand(listOfEnergyFields)
  aibot.target = aibot.targetEnergyField
  true
end

#bot 1 search and draw over enemy colors
#bot 2 when low: search energy felds suchen, else: draw non owned fields
#bot 3 draw all non owned fields

function searchForNotOwnedFields(aibot::AIBot)
  current = aibot.node
  for i=1:length(world) # do not go to infinity
    for node in current.neighbors
      if node.blocked || node.owner == playerNumber continue end
      return node
    end
    current=current.parent
  end
  nothing
end

function searchNext(aibot::AIBot)
  # when target reached -> get new target
  if aibot.target != nothing && aibot.target.owner == playerNumber aibot.target = nothing end

  waypoints = listOfWaypoints[aibot.parent.id]
  if length(waypoints)>0
    aibot.target = popfirst!(waypoints)
    return
  end

  if aibot.parent.id != 0
    if rechargeWhenLow(aibot)
      if aibot.parent.id != 2
        next = aStarSearch(aibot.node,aibot.targetEnergyField)
        if next != nothing
          for n in next println("$(aibot.parent.id): $(n.position) => $(aibot.targetEnergyField.position)") end
          waypoints = listOfWaypoints[aibot.parent.id]=next
          aibot.target = popfirst!(waypoints)
        end #aibot.target
      end
      return # ignore all nodes an go straight to energy field
    else
      #target = searchForNotOwnedFields(aibot)
      #if target == nothing aibot.targetEnergyField end
      if aibot.target != aibot.targetEnergyField && aibot.targetEnergyField != nothing
        global ENERGYTIME = time()+10 # get next 10 sec
        aibot.targetEnergyField = nothing
      end
    end
    return
  else
    return
  end

  #next = aStarSearch(aibot.node,aibot.targetEnergyField)
  #if next != nothing next=next[2] end

  next = nothing
  current = aibot.node
  passed = []

  # bot 1
  #while next == nothing
  for i=1:length(world) # do not go to infinity
    previous = current
    for node in current.neighbors
      if in(node,passed) || (aibot.parent.id != 1 && node.blocked) continue end
      current=node
      break
    end
    if current == previous
      push!(passed,current)
      current=current.parent
      if current == nothing break end # end of tree?
    end
    if current.owner != aibot.parent.id && (aibot.parent.id == 3 || (aibot.parent.id == 1 && current.owner>0))
      next=current
      break
    end # break
  end
  #if next == nothing @warn "$(aibot.parent.id): No target found!" end
  aibot.target = next
end

#=
function searchNext_old(aibot::AIBot)
  if aibot.node == nothing return end
  #if aibot.parent.id != 1 aibot.target=aibot.node; return end
  #return
  #if !charge
    if aibot.target != nothing && aibot.target.owner == playerNumber aibot.target = nothing end
    #distance(aibot.parent.position,aibot.target) < 0.051 #
    if aibot.target == nothing
      (best,bestdist)=(nothing,999)
      for node in aibot.node.neighbors
        if aibot.parent.id != 2 && node.blocked continue end # skip blocked
        if node.owner == aibot.parent.id continue end # skip owned
        if in(node,aibot.avoid) || in(node,aibot.history) continue end
        if aibot.parent.id != 1 best=node
        else
          x = distance(node,aibot.targetEnergyField)
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
=#

function updateWorld(client::NetworkClient)
  global world, bsave
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
  serverTimeout = 1
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
    global ENERGYTIME = startTime + 10
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
