printStackTrace(ex) = Base.showerror(stdout, ex, catch_backtrace())

println("Start Server...")
while true
  try
    run(`java -jar cywwtaip.jar`)
  catch ex
    printStackTrace(ex)
  end
  println("Start next session in 3 sec...")
  sleep(3)
end
