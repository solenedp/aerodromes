using JuMP, Gurobi


include("read.jl")
n,d,f,Amin,Nr,R,regions,coords,D = readInstance("instances aerodromes-20221003/aerodrome_20_1.txt")
model = Model(Gurobi.Optimizer)
#déclaration des variables
@variable(model, x[1:n,1:n], Bin) #variables binaires -> présence ou non de l'arc i,j
@variable(model,u[2:n]) #variable entières : numéro du sommet i dans le chemin
#déclaration des contraintes
A = Matrix{Int64}(ones(n,n))
b = [1 for i in 1:n]
c = [0 for i in 1:n]
c[d]=1
c[f]=-1
@constraint(model,x*b .<=b)
@constraint(model, transpose(x)*b .<=b)
@constraint(model,x*b+transpose(x)*b .== c)
@constraint(model,sum(sum(x[i,j] for j in 1:n) for i in 1:n)>= Amin-1)
for i in 1:n 
    for j in 1:n 
        if D[i,j]>R 
            @constraint(model,x[i,j]==0)
        end
    end
end

for i in eachindex(regions) 
    @constraint(model, sum(sum(x[k,j] for j in 1:n) for k in regions[i] ) == 1)
end
for i in 2:n
    for j in 2:n
        @constraint(model, u[j]>=u[i]+1-Amin*(1-x[i,j]))
    end
end

#objectif
@objective(model, Min, sum(sum(D[i,j]*x[i,j] for i in 1:n) for j in 1:n))


JuMP.optimize!(model)
objective_value = JuMP.objective_value(model)
println("Objective value : ",objective_value)