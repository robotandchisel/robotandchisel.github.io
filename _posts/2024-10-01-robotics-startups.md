---
layout: post
title: Some Thoughts on Robotics Startups
description: A bit of a different type of blog post.
date: '2024-10-01T14:30:00.00-04:00'
author: Michael Ferguson
tags:
- robots
---

<i>This is not investment advice. <br>
This probably shouldn't be construed as advice of any kind.</i>

## Robotics Startups of the Past and Present

Over the past year or two, I've had a number of conversations that were basically
<i>"why is it so much harder to get funding for robotics stuff these days?"</i>.
While you could blame macroeconomics, lack of LP liquidity, and higher interest
rates, I think this actually largely comes down to the fact that there haven't
been all that many really big exits in robotics. Robotics founders tend to be
very good at building cool tech solutions, but not so good at actually selling
and monetizing that technology.

When Kiva Systems sold to Amazon in 2012 for $775M, there weren't all that many robotics
companies out there - so it looked like maybe this was a space that would generate
great returns. Now, a little over 10 years later, there have been a handful of exits
in the several-hundred-million-USD range (Universal Robots, 6Rivers, MIR, Fetch Robotics, Clearpath Robotics / Otto),
but most of those raised significantly more capital than Kiva Systems did and all of them
exited for a lower price than Kiva. None of them exited at the same kind of crazy
valuation that Google bought Nest for in 2014 ($3.2B - this was another "hardware"
exit that drove interest in robotics around that time).

Universal Robots and Mobile Industrial Robots (MIR) are setup more like traditional robot
manufacturers - they build robots, sell them at relatively low margins, and use an extensive
network of integrators who actually install and program the robots. Notably, both companies
come out of the EU - I feel that most US-based investors who do seed/A rounds would not
fund this sort of company today.

What those US-based investors do fund looks a bit different. These companies are largely
founded by robotics experts. They are vertically integrated, often manufacturing their own
robots in-house, building their own mapping, localization, navigation, and cloud-based
fleet management, and often setting up their own direct sales channels. Many of those
direct sales channels focus on "recurring revenue" by offering robots as a service.
There are numerous examples.

One outlier to this model though would be Locus Robotics. They weren't founded by robotics
experts - they were founded by domain experts in third party logistics (3PL). These domain
experts were also a built-in customer - as an early customer of Kiva Systems, [they had a 
real need for a new robotics solution after the Kiva acquisition](https://spectrum.ieee.org/locus-robotics-warehouse-automation-robots)
And hey, guess what, Locus is doing pretty well.

## Robotics is not SAAS

What surprises me is there are still investment firms trying this same playbook today.
They go out and fund a robotics company founded entirely by roboticsists (and for some
reason, many seem to think there are bonus points if every one of the founders is a 
Robotics PhD and has never held a job outside of academia).

Some of these firms claim that RaaS is the new SaaS: spoiler alert - it's probably not!
Robotics is capital intensive. Especially if you are buying all these robots, keeping them
on the books, and then renting them out with a 1-2 year payback window. Even worse:
robots age a lot worse than servers.

The really tricky thing about "robotics startups" is that the robot is JUST ONE PART
of the business / product.
It's really just the starting point - and you will eventually end up expending
far more effort on the rest of the product: the software you need for deployment and
monitoring, the sales organization, the integration teams.

Finally, while many folks will tell you that <i>"hardware is hard"</i> - the bigger problem is that
hardware is SLOW. Supply chains have improved from the days of the pandemic, but
they are still slow, inefficient and generally a bit of a hot mess. So when you do suddenly
land all those orders - good luck getting the parts you need to actually fulfill the order
quickly.

/rant

## Robotics Ecosystems

And now we get to something maybe actually useful to somebody. If our current generation of
robotic startups are fully vertically integrated and founded solely by robotics people,
what does the next generation look like?

Much of the really interesting stuff with computers and the internet started to happen when
people who weren't just computer nerds were able to build companies in the space. I
think the same thing could happen for robotics.

These next generation robotics companies will have a founding team with domain experts
in whatever problem the robot is solving. These companies probably won't even be called
robotics companies. They'll be healthcare automation startups, or 3PL startups, etc.

They probably won't be fully vertically integrated, instead choosing to use more off
the shelf hardware and software components.

We already see some of this happening today - in the early days of the RoboBusiness
conference, all sorts of "robot" companies exhibited - today, those robot companies
put much more emphasis in the tradeshows for their industry - show likes ProMat
or Modex for warehouse logistics
providers. The majority of exhibitors at robotics conferences are now selling
(largely hardware) components to robot companies.

With an ecosystem of more focused next-generation robotics companies, these startups
won't have to build everything in-house. Companies like
[InOrbit](https://www.inorbit.ai/),
[Formant](https://formant.io/) and
[Foxglove](https://foxglove.dev/)
exist today and offer a slice of tools needed to build a robotics solution. You can buy
robots from UR and MIR. The ROS 2 variant of navigation, Nav2, and arm planning (MoveIt2)
are already being used in 
commercial products with far less customization than was needed in ROS 1 - and their 
respective supporting  companies ([Open Navigation LLC](https://www.opennav.org/) and
[PickNik](https://picknik.ai/)) exist to help next-generation 
robotics companies leverage these open source projects.

## The Actionable Stuff

Numerous people have asked for startup advice over the years - I have often, wrongly,
focused on very narrow things (don't go cheap on lawyers, etc).

I'm not sure I'll ever do another robotics company, but here is what my dream
founder team would look like for a future robotics startup:

 * CEO - A domain expert in whatever industry you are selling into. Significant
   product experience. Should be able to sell - the CEO's industry connections
   will basically take the place of hiring a sales team initially.
 * CTO - Robotics expert, with product experience. In order to properly lead
   the (hopefully relatively small) engineering organization, and integration
   of Off-The-Shelf (OTS) components and vendors, the CTO will need a solid grasp
   of hardware, web/enterprise software, and any other product specific technologies.

Put that team together and then find a real product need - the simpler the better.
Robotics people love to over-complicate things. Do you really need a mobile base?
Do you really need an arm? Or is there some simpler automation solution that you
should be tackling?
