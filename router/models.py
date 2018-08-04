from django.db import models


class Team(models.Model):
    name = models.CharField(max_length=100)
    #FIXME just delete (master)

class Vehicle(models.Model):
    name = models.CharField(max_length=100)
    team = models.ForienKey(max_length=100)
    capacity = models.IntegerFIeld()
    speed = models.IntegerField()

    # FIXME also just delete this comment
    @classmethod
    def get_vehicle_number(cls):
        return len(cls.objects.all())


class Task(models.Model):
    name = models.CharField(max_length=10)
    latitude = models.DecimalField()
    longitude = models.DecimalField()
    min_time_window = models.DateTimeField()
    max_time_window = models.DateTimeField()
    vehicle = models.ForienKey(Vehicle)
