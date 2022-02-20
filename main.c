#include <stdio.h>
#include <raylib.h>
#include <raymath.h>

void reverseArrayFloats(float arr[], size_t arrCount);
void reverseArrayVector2s(Vector2 arr[], size_t arrCount);
void solveIk(Vector2 points[], size_t pointsCount, Vector2 target);

#define MAX_POINTS 100
#define POINT_RADIUS 10

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    int screenWidth = 1024;
    int screenHeight = 600;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "FABRIK Forward And Backward Inverse Kinematic");

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    Vector2 points[MAX_POINTS];
    Vector2 target;
    int pointsCount = 0;
    bool inDrawingMode = true;
    int selectedPoint = -1;

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        if (IsWindowResized()) {
            screenWidth = GetScreenWidth();
            screenHeight = GetScreenHeight();
        }
        // Update
        //----------------------------------------------------------------------------------
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && pointsCount) {
            inDrawingMode = !inDrawingMode;
        }
        if (IsKeyPressed(KEY_DELETE) && pointsCount) {
            if (selectedPoint != -1) {
                for (int i = selectedPoint + 1; i < pointsCount; i++) {
                    points[i - 1] = points[i];
                }
            }
            --pointsCount;
        }
        if (!inDrawingMode && pointsCount) {
            target = GetMousePosition();
            solveIk(points, pointsCount, target);
        } else {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                points[pointsCount] = GetMousePosition();
                ++pointsCount;
            } else {
                selectedPoint = -1;
                for (int i = 0; i < pointsCount; i++) {
                    if (CheckCollisionPointCircle(GetMousePosition(), points[i], POINT_RADIUS)) {
                        selectedPoint = i;
                        break;
                    }
                }
            }
        }
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        DrawFPS(screenWidth - 80, 5);
        DrawText("Left click to place point\nRight click to start/stop the IK simulation\nDelete Key to delete last point", 5, 5, 20, LIGHTGRAY);

        for (size_t i = 0; i < pointsCount; i++)
        {
            if (i < pointsCount - 1) {
                DrawLineEx(points[i], points[i+1], 4, GRAY);
            }
            Color pointColor = BLACK;
            if (i == selectedPoint) {
                pointColor = RED;
            }
            DrawCircle((int)points[i].x, (int)points[i].y, POINT_RADIUS, pointColor);
        }

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

void reverseArrayFloats(float *arr, size_t arrCount) {
    float rev[arrCount];

    for (size_t i = 0; i < arrCount; i++)
    {
        rev[arrCount - 1 - i] = arr[i];
    }

    for (size_t i = 0; i < arrCount; i++)
    {
        arr[i] = rev[i];
    }
}

void reverseArrayVector2s(Vector2 *arr, size_t arrCount) {
    Vector2 rev[arrCount];

    for (size_t i = 0; i < arrCount; i++)
    {
        rev[arrCount - 1 - i] = arr[i];
    }

    for (size_t i = 0; i < arrCount; i++)
    {
        arr[i] = rev[i];
    }
}

void solveIk(Vector2 *points, size_t pointsCount, Vector2 target) {
    int maxIterations = 100;
    float minAcceptableDst = 0.01f;

    Vector2 origin = points[0];
    float segmentLengths[pointsCount - 1];

    for (size_t i = 0; i < pointsCount - 1; i++)
    {
        segmentLengths[i] = Vector2Length(Vector2Subtract(points[i+1], points[i]));
    }

    for (int iteration = 0; iteration < maxIterations; iteration ++)
    {
        bool startingFromTarget = iteration % 2 == 0;
        reverseArrayFloats(segmentLengths, pointsCount - 1);
        reverseArrayVector2s(points, pointsCount);

        points[0] = startingFromTarget ? target : origin;

        for (size_t i = 1; i < pointsCount; i ++)
        {
            Vector2 dir = Vector2Normalize(Vector2Subtract(points[i], points[i-1]));
            points[i] = Vector2Add(points[i-1],Vector2Multiply( dir, (Vector2){segmentLengths[i-1], segmentLengths[i-1]}));
        }

        float dstToTarget = Vector2Length(Vector2Subtract(points[pointsCount - 1], target));
        if (! startingFromTarget && dstToTarget <= minAcceptableDst) {
            return;
        }
    }
}



