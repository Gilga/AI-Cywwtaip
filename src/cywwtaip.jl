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
  JavaCall.destroy() # note: JavaCall cannot be initalized again in same process
  println("--------------------------------------------------------------------------------")
end

####################################################################################################

@info "JavaCall Imports..."
#jlm = @jimport "java.lang.Math"
JNetworkClient = @jimport lenz.htw.cywwtaip.net.NetworkClient
JGraphNode = @jimport lenz.htw.cywwtaip.world.GraphNode

####################################################################################################

@info "JavaCall Definitions..."

mutable struct GraphNode
  handle::JGraphNode

  x::Float32
  y::Float32
  z::Float32
  
  neighbors::Array{JGraphNode, 1}
  blocked::Bool
  owner::Int32
  
  function GraphNode(node::JGraphNode)
    this = new(node)
    this.x = jfield(node, "x", jfloat)
    this.y = jfield(node, "y", jfloat)
    this.z = jfield(node, "z", jfloat)
    this.blocked = Bool(jfield(node, "blocked", jboolean))
    this.owner = jfield(node, "owner", jint)
    this.neighbors = jfield(node, "neighbors", Array{JGraphNode, 1})
    this
  end
end
export GraphNode

@info "JavaCall Hooks..."
hashCode(node::JGraphNode) = jcall(node, "hashCode", jint, ())
equals(node::JGraphNode, obj::JObject) = Bool(jcall(node, "equals", jboolean, (JObject,), obj))
toString(node::JGraphNode) = jcall(node, "toString", JString, ())
toString(node::GraphNode) = toString(node.handle)

NetworkClient(hostname::String, teamName::String, winMsg::String) = JNetworkClient((JString,JString,JString,), "localhost", "sds", "dsdsds")
isAlive(client::JNetworkClient) = Bool(jcall(client, "isAlive", jboolean, ()))
getMyPlayerNumber(client::JNetworkClient) = jcall(client, "getMyPlayerNumber", jint, ())
getScore(client::JNetworkClient, player::Integer) = jcall(client, "getScore", jint, (jint,), Int32(player))
getBotPosition(client::JNetworkClient, player::Integer, bot::Integer) = jcall(client, "getBotPosition", Array{jfloat, 1}, (jint,jint,), Int32(player), Int32(bot)) #array
getBotDirection(client::JNetworkClient, bot::Integer) = jcall(client, "getBotDirection", Array{jfloat, 1}, (jint,), Int32(bot))
getBotSpeed(client::JNetworkClient, bot::Integer) = jcall(client, "getBotSpeed", jfloat, (jint,), Int32(bot))
changeMoveDirection(client::JNetworkClient, bot::Integer, angle::AbstractFloat) = jcall(client, "changeMoveDirection", Nothing, (jint,jfloat,), Int32(bot), Float32(angle))
getGraph(client::JNetworkClient) = (node->GraphNode(node)).(jcall(client, "getGraph", Array{JGraphNode, 1}, ()))

hashCode(client::JNetworkClient) = jcall(client, "hashCode", jint, ())
equals(client::JNetworkClient, obj::JObject) = Bool(jcall(client, "equals", jboolean, (JObject,), obj))
toString(client::JNetworkClient) = jcall(client, "toString", JString, ())

####################################################################################################

export hashCode
export equals
export toString

export NetworkClient
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
Base.show(io::IO, this::JGraphNode) = print(io, string("$(toString(this))"))
Base.show(io::IO, this::JNetworkClient) = print(io, string("$(toString(this))"))

####################################################################################################

end # cywwtaip
