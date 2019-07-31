module cywwtaip

using Base.Threads
using Distributed

using JavaCall

####################################################################################################

JAR_FILE = joinpath(@__DIR__,"../","cywwtaip.jar")

####################################################################################################

function init()
  if JavaCall.isloaded() return end
  println("--------------------------------------------------------------------------------")
  @info "JavaCall Init..."
  JavaCall.init(["-Xmx512M", "-Djava.class.path=.;$(JAR_FILE)", "-verbose:jni", "-verbose:gc"])
  println("--------------------------------------------------------------------------------")
end

function destroy()
  println("--------------------------------------------------------------------------------")
  @info "Destroy JavaCall"
  #JavaCall.destroy()
  @warn "Disabled because of this problem: If destroyed, JavaCall cannot be initalized again in same process!"
  println("--------------------------------------------------------------------------------")
end

####################################################################################################

@info "JavaCall Imports..."
#jlm = @jimport "java.lang.Math"
JNetworkClient = @jimport lenz.htw.cywwtaip.net.NetworkClient
JGraphNode = @jimport lenz.htw.cywwtaip.world.GraphNode

####################################################################################################

@info "JavaCall Classes..."

mutable struct NetworkClient
  handle::JNetworkClient
  NetworkClient(handle::JNetworkClient) = new(handle)
end
export NetworkClient

const Position = Array{Float32, 1}
const Direction = Array{Float32, 1}

mutable struct Bot
  id::Int32
  player::Int32
  position::Position
  direction::Direction
  speed::Float32

  Bot(id::Integer, player::Integer, position::Position, direction::Direction, speed::Float32) = new(id, player, position, direction, speed)

  function Bot(client::NetworkClient, id::Integer, player::Integer)
    this = new(id, player)
    this.position = getBotPosition(client, player, id)
    this.direction = getBotDirection(client, id)
    this.speed = getBotSpeed(client, id)
    this
  end
end
export Bot

mutable struct GraphNode
  id::UInt32
  handle::Union{Nothing, JGraphNode}
  position::Array{jfloat, 1}
  parent::Union{Nothing, GraphNode}
  neighbors::Array{Union{Nothing, GraphNode, JGraphNode}, 1}
  blocked::Bool
  owner::Int32

  function GraphNode(id::Integer=0; handle::Union{Nothing, JGraphNode}=nothing)
    this = new(id, handle)
    this.parent = nothing
    if handle != nothing parse(this, handle) end
    this
  end
end
export GraphNode

# TEMPLATE ARRAY
TEMPLATE_LIST = Array{GraphNode,1}(undef, 40962)
@threads for i=1:length(TEMPLATE_LIST) TEMPLATE_LIST[i] = GraphNode(i) end

const DEFAULT_NODE_FUNCTION=(x)->x

getWorldTemplate() = Array{Union{Nothing,GraphNode},1}(undef,length(TEMPLATE_LIST))
export getWorldTemplate

function reset()
  #global TEMPLATE_LIST
  #@sync @distributed for i=1:length(TEMPLATE_LIST) TEMPLATE_LIST[i] = GraphNode() end
end

parse(this::JGraphNode) = GraphNode(0; handle=this)

function parse(this::GraphNode, handle::JGraphNode)
  this.handle = handle
  this.position = getPosition(handle) #[jfield(handle, "x", jfloat), jfield(handle, "y", jfloat), jfield(handle, "z", jfloat)]
  this.blocked = Bool(jfield(handle, "blocked", jboolean))
  this.owner = jfield(handle, "owner", jint)
  this.neighbors = jfield(handle, "neighbors", Array{JGraphNode, 1})
  this
end

function parse(handles::Array{JGraphNode, 1}; f::Function=DEFAULT_NODE_FUNCTION)
  nodes = TEMPLATE_LIST #deepcopy(TEMPLATE_LIST)
  @time @sync @distributed for i=1:length(handles) f(parse(nodes[i], handles[i])) end #node in nodes, handle in handles
  nodes
end

export parse

@info "JavaCall Hooks..."

NetworkClient(hostname::String, teamName::String, winMsg::String, timeout::Int=30) = NetworkClient(JNetworkClient((JString,JString,JString,jint), hostname, teamName, winMsg, timeout))

isAlive(this::NetworkClient) = Bool(jcall(this.handle, "isAlive", jboolean, ()))
getMyPlayerNumber(this::NetworkClient) = jcall(this.handle, "getMyPlayerNumber", jint, ())+1

getScore(this::NetworkClient, player::Integer) = jcall(this.handle, "getScore", jint, (jint,), Int32(player-1))
getScore(this) = [getScore(this,1),getScore(this,2),getScore(this,3)]

getBotPosition(this::NetworkClient, player::Integer, bot::Integer) = jcall(this.handle, "getBotPosition", Array{jfloat, 1}, (jint,jint,), Int32(player-1), Int32(bot-1)) #array
getBotDirection(this::NetworkClient, bot::Integer) = jcall(this.handle, "getBotDirection", Array{jfloat, 1}, (jint,), Int32(bot-1))
getBotSpeed(this::NetworkClient, bot::Integer) = jcall(this.handle, "getBotSpeed", jfloat, (jint,), Int32(bot-1))

getBotPositions(this::NetworkClient, player::Integer) = [getBotPosition(this, player, 1),getBotPosition(this, player, 2),getBotPosition(this, player, 3)]
getBotDirections(this::NetworkClient) = [getBotDirection(this, 1),getBotDirection(this, 2),getBotDirection(this, 3)]
getBotSpeeds(this::NetworkClient) = [getBotSpeed(this, 1),getBotSpeed(this, 2),getBotSpeed(this, 3)]

getBots(this::NetworkClient, player::Integer) = [Bot(this, 1, player),Bot(this, 2, player),Bot(this, 3, player)]
getBots(this::NetworkClient) = [getBots(this, 1),getBots(this, 2),getBots(this, 3)]

function updateBot(this::NetworkClient, bot::Bot)
  bot.position = getBotPosition(this, bot.player, bot.id)
  bot.direction = getBotDirection(this, bot.id)
  bot.speed = getBotSpeed(this, bot.id)
end

updateBots(this::NetworkClient, bots::Array{Bot,1}) = for bot in bots updateBot(this,bot) end
updateBots(this::NetworkClient, playerbots::Array{Array{Bot,1},1}) = for bots in playerbots updateBots(this,bots) end

changeMoveDirection(this::NetworkClient, bot::Integer, angle::AbstractFloat) = jcall(this.handle, "changeMoveDirection", Nothing, (jint,jfloat,), Int32(bot-1), Float32(angle))

function getGraph(this::NetworkClient)
  handles = jcall(this.handle, "getGraph", Array{JGraphNode, 1}, ())
  parse(handles)
end

function update(this::GraphNode, handle::JGraphNode)
  #this.handle = handle # not used, can be ignored
  #this.position # should not change (is static anyway)
  this.blocked = Bool(jfield(handle, "blocked", jboolean)) # can change
  this.owner = jfield(handle, "owner", jint) # can change
  #this.neighbors # should not change, we replaced it previously in createWorld()
  this
end

function updateGraph(this::NetworkClient, nodes::Array{GraphNode,1})
  handles = jcall(this.handle, "getGraph", Array{JGraphNode, 1}, ())
  @threads for i=1:length(handles) update(nodes[i], handles[i]) end
end

hashCode(this::NetworkClient) = jcall(this.handle, "hashCode", jint, ())
equals(this::NetworkClient, obj::JObject) = Bool(jcall(this.handle, "equals", jboolean, (JObject,), obj))
toString(this::NetworkClient) = jcall(this.handle, "toString", JString, ())

hashCode(this::GraphNode) = jcall(this.handle, "hashCode", jint, ())
equals(this::GraphNode, obj::JObject) = Bool(jcall(this.handle, "equals", jboolean, (JObject,), obj))
toString(this::GraphNode) = jcall(this.handle, "toString", JString, ())
toInfoString(this::GraphNode) = "Node["*string(this.id)*"] owner="* string(this.owner)*", blocked="*string(this.blocked)*", pos=["*string(join(this.position,", "))*"]"

getPosition(this::JGraphNode) = [jfield(this, "x", jfloat), jfield(this, "y", jfloat), jfield(this, "z", jfloat)]

####################################################################################################

export Position
export Direction

export hashCode
export equals
export toString
export toInfoString

export isAlive
export getMyPlayerNumber
export getScore
export getBotPosition
export getBotDirection
export getBotSpeed
export getBotPositions
export getBotDirections
export getBotSpeeds
export getBots
export changeMoveDirection
export getGraph
export updateGraph
export updateBot
export updateBots
export getPosition

####################################################################################################

Base.show(io::IO, this::GraphNode) = print(io, string("$(toString(this))"))
Base.show(io::IO, this::NetworkClient) = print(io, string("$(toString(this))"))

####################################################################################################

end # cywwtaip
