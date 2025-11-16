CS 639 final project

Our goal was to demonstrate throwing and catching with robot arms (UR5e).
For throwing, we have a human defined throwing policy and one that was learned using trajectory optimization.
To demonstrate catching, we had an arm track the projectile motion of the block and move to intercept the block.

Install dependencies with

```bash
pip isntall -r requirements.txt
```

You can run our code with the following command

```bash
webots worlds/final.wbt
```

Or select `worlds/final.wbt` from Webots in File > Open World...