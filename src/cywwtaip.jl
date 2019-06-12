module cywwtaip

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

mutable struct GraphNode
  handle::Union{Nothing, JGraphNode}
  position::Array{jfloat, 1}
  neighbors::Array{JGraphNode, 1}
  blocked::Bool
  owner::Int32

  GraphNode() = new(nothing, [], [], false, -1)

  function GraphNode(handle::JGraphNode)
    this = new(handle)
    this.position = [jfield(handle, "x", jfloat), jfield(handle, "y", jfloat), jfield(handle, "z", jfloat)]
    this.blocked = Bool(jfield(handle, "blocked", jboolean))
    this.owner =  jfield(handle, "owner", jint)
    this.neighbors = jfield(handle, "neighbors", Array{JGraphNode, 1})
    this
  end
end
export GraphNode

# TEMPLATE ARRAY
TEMPLATE_LIST = Array{GraphNode,1}(undef, 4096)

function reset()
  global TEMPLATE_LIST
  for i=1:length(TEMPLATE_LIST) TEMPLATE_LIST[i] = GraphNode() end
end

function translate(this::GraphNode, handle::JGraphNode)
  this.position = [jfield(handle, "x", jfloat), jfield(handle, "y", jfloat), jfield(handle, "z", jfloat)]
  this.blocked = Bool(jfield(handle, "blocked", jboolean))
  this.owner = jfield(handle, "owner", jint)
  this.neighbors = jfield(handle, "neighbors", Array{JGraphNode, 1})
  this
end

translate(handle::JGraphNode) = GraphNode(handle)

function translate(handles::Array{JGraphNode, 1})
  nodes = deepcopy(TEMPLATE_LIST)
  for (node,handle) in zip(nodes,handles) translate(node, handle) end
  nodes
end

export translate

@info "JavaCall Hooks..."

NetworkClient(hostname::String, teamName::String, winMsg::String) = NetworkClient(JNetworkClient((JString,JString,JString,), hostname, teamName, winMsg))

isAlive(client::NetworkClient) = Bool(jcall(client.handle, "isAlive", jboolean, ()))
getMyPlayerNumber(client::NetworkClient) = jcall(client.handle, "getMyPlayerNumber", jint, ())
getScore(client::NetworkClient, player::Integer) = jcall(client.handle, "getScore", jint, (jint,), Int32(player))
getScore(client) = [getScore(client,0),getScore(client,1),getScore(client,2)]

getBotPosition(client::NetworkClient, player::Integer, bot::Integer) = jcall(client.handle, "getBotPosition", Array{jfloat, 1}, (jint,jint,), Int32(player), Int32(bot)) #array
getBotDirection(client::NetworkClient, bot::Integer) = jcall(client.handle, "getBotDirection", Array{jfloat, 1}, (jint,), Int32(bot))
getBotSpeed(client::NetworkClient, bot::Integer) = jcall(client.handle, "getBotSpeed", jfloat, (jint,), Int32(bot))
getBotPosition(client, player) = [getBotPosition(client, player, 0),getBotPosition(client, player, 1),getBotPosition(client, player, 2)]
getBotDirection(client, player) = [getBotDirection(client, player, 0),getBotDirection(client, player, 1),getBotDirection(client, player, 2)]
getBotSpeed(client, player) = [getBotSpeed(client, player, 0),getBotSpeed(client, player, 1),getBotSpeed(client, player, 2)]
changeMoveDirection(client::NetworkClient, bot::Integer, angle::AbstractFloat) = jcall(client.handle, "changeMoveDirection", Nothing, (jint,jfloat,), Int32(bot), Float32(angle))

function getGraph(client::NetworkClient)
  handles = jcall(client.handle, "getGraph", Array{JGraphNode, 1}, ())
  translate(handles)
end

hashCode(client::NetworkClient) = jcall(client.handle, "hashCode", jint, ())
equals(client::NetworkClient, obj::JObject) = Bool(jcall(client.handle, "equals", jboolean, (JObject,), obj))
toString(client::NetworkClient) = jcall(client.handle, "toString", JString, ())

hashCode(node::GraphNode) = jcall(node.handle, "hashCode", jint, ())
equals(node::GraphNode, obj::JObject) = Bool(jcall(node.handle, "equals", jboolean, (JObject,), obj))
toString(node::GraphNode) = jcall(node.handle, "toString", JString, ())
toInfoString(node::GraphNode) = "GraphNode "*string(node.position) * ": owner=" * string(node.owner) * ", blocked=" * string(node.blocked)

####################################################################################################

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
export changeMoveDirection
export getGraph

####################################################################################################

Base.show(io::IO, this::GraphNode) = print(io, string("$(toString(this))"))
Base.show(io::IO, this::NetworkClient) = print(io, string("$(toString(this))"))

####################################################################################################

end # cywwtaip
