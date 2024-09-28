from django.urls import path
from . import views

urlpatterns = [
    path('', views.index, name='index'),  # Root page
    path('move/', views.move, name='move'),  # Drive commands
    path('mower_control/', views.mower_control, name='mower_control'),  # Mower control
    path('dethatcher_control/', views.dethatcher_control, name='dethatcher_control'),  # Dethatcher control
    path('weed_control/', views.weed_control, name='weed_control'),  # Weed control
    path('aerator_control/', views.aerator_control, name='aerator_control'),  # Aerator control
    path('battery_control/', views.battery_control, name='battery_control'),  # Battery control
    path('system_status/', views.system_status, name='system_status'),  # System status endpoint
    # path('logs/', views.get_logs, name='get_logs'),  # Corrected from `views.logs` to `views.get_logs` 
]
