funkcionalan program koji će se koristiti na shiftu.

lampe su adresirane FARO100-1, FARO100-2, RDD200-1, RDD200-2 ili all za upravljanje svima isto tako ih je potrebno i poredati

POST https://pp21znl43a.execute-api.us-east-1.amazonaws.com/dev/command
{
    "deviceId": "FARO100-1",
    "command": "breathe w",
    "R": 255,
    "G": 200,
    "B": 200,
    "W": 100,
    "beamX": 128,
    "beamY": 64
}

