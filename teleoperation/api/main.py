from fastapi import FastAPI
import control.router
import sensors.router
from lifespan import lifespan

app = FastAPI(lifespan = lifespan, title = "Robot Control API")


app.include_router(control.router, prefix="/control", tags=["control"])
app.include_router(sensors.router, prefix="/sensors", tags=["sensors"])

@app.get("/health", tags=["health"])
async def root():
    return "ok"


