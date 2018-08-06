from .base import *

# SECURITY WARNING: keep the secret key used in production secret!
SECRET_KEY = env('DJANGO_SECRET_KEY', default='r=s4zpbeq^ua&gzxps%urj2q)gu7g4&ubl0_e-gwjptuq!*&^c')

DEBUG = env.bool('DJANGO_DEBUG', default=True)
